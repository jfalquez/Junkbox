#include <pangolin/pangolin.h>
#include <pangolin/glcuda.h>
#include <kangaroo/kangaroo.h>
#include <sophus/se3.h>
#include <kangaroo/../applications/common/ImageSelect.h>
#include <SceneGraph/SceneGraph.h>
#include <RPG/Devices.h>
#include <Mvlpp/Mvl.h>
#include "ParseArgs.h"

using namespace std;
namespace pango = pangolin;
namespace sg = SceneGraph;

int main(int argc, char** argv)
{
    // parse parameters
    CameraDevice* pCam = ParseArgs( argc, argv );

    // read camera model file
    std::string sCamModFileName = pCam->GetProperty( "CamModelFile", "rcmod.xml" );
    mvl::CameraModel CamModel( sCamModFileName );

    // vector of images captured
    vector< rpg::ImageWrapper > vImages;

    // initial capture for image properties
    pCam->Capture( vImages );

    // image properties
    const unsigned int nImgWidth = vImages[1].Image.cols;
    const unsigned int nImgHeight = vImages[0].Image.rows;

    // create a GUI window
    pangolin::CreateGlutWindowAndBind("Dense Tracker",nImgWidth*2,nImgHeight*2);
    glewInit();
    cudaGLSetGLDevice(0);
    SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();

    // Define Camera Render Object (for view / scene browsing)
    pango::OpenGlRenderState glState( pango::ProjectionMatrix( 640, 480, 420, 420, 320, 240, 0.1, 1000 ),
                                      pango::ModelViewLookAt( -6, 0, -10, 1, 0, 0, pango::AxisNegZ ) );

    // Scenegraph to hold GLObjects and relative transformations
    sg::GLSceneGraph glGraph;

    // We define a new view which will reside within the container.
    pango::View glView3D;

    // We set the views location on screen and add a handler
    glView3D.SetHandler( new sg::HandlerSceneGraph( glGraph, glState, pango::AxisNegZ ) );
    glView3D.SetDrawFunction( sg::ActivateDrawFunctor( glGraph, glState ) );

    // create a side panel
    pango::CreatePanel("ui").SetBounds(0,1,0,pangolin::Attach::Pix(180));
    pango::Var<int> nMaxDisparity("ui.MaxDisp",10,0,100);
    pango::Var<float> fNormC("ui.Norm c",10,0,10);

    // create a view container
    pangolin::View& guiContainer = pangolin::CreateDisplay()
            .SetBounds(0,1,pangolin::Attach::Pix(180),1)
            .SetLayout(pangolin::LayoutEqual);
    const unsigned int nNumConts = 3;
    for( unsigned int ii = 0; ii <  nNumConts; ii ++ ) {
        pangolin::View& v = pangolin::CreateDisplay();
        v.SetAspect((double)nImgWidth/nImgHeight);
        guiContainer.AddDisplay(v);
    }

    // add 3d view to container
    guiContainer.AddDisplay(glView3D);

    // draw grid on 3D window
    sg::GLGrid glGrid;
    glGraph.AddChild( &glGrid );

    // gl axis
    sg::GLAxis glCurPose;
    sg::GLAxis glPrevPose;
    glGraph.AddChild(&glCurPose);

    // initialize vbo's
    pango::GlBufferCudaPtr vbo( pango::GlArrayBuffer, nImgWidth, nImgHeight, GL_FLOAT, 4,
                                cudaGraphicsMapFlagsWriteDiscard, GL_STREAM_DRAW );
    pango::GlBufferCudaPtr cbo( pango::GlArrayBuffer, nImgWidth, nImgHeight, GL_UNSIGNED_BYTE, 4,
                                cudaGraphicsMapFlagsWriteDiscard, GL_STREAM_DRAW );
    pango::GlBufferCudaPtr ibo( pango::GlElementArrayBuffer, nImgWidth, nImgHeight, GL_UNSIGNED_INT, 2 );

    // create vbo
    SceneGraph::GLVbo glVBO( &vbo, &ibo, &cbo );

    // Generate Index Buffer Object for rendering mesh
    {
        pango::CudaScopedMappedPtr var( ibo );
        Gpu::Image< uint2 >        dIbo( (uint2*)*var, nImgWidth, nImgHeight );
        Gpu::GenerateTriangleStripIndexBuffer( dIbo );
    }
    glGraph.AddChild( &glVBO );
    glVBO.AddChild(&glPrevPose);

    Gpu::Image<unsigned char, Gpu::TargetDevice, Gpu::Manage> dLeftImg(nImgWidth, nImgHeight);
    Gpu::Image<unsigned char, Gpu::TargetDevice, Gpu::Manage> dRightImg(nImgWidth, nImgHeight);
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage > dDispInt( nImgWidth, nImgHeight );
    Gpu::Image< float, Gpu::TargetDevice, Gpu::Manage >         dDisp( nImgWidth, nImgHeight );

    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage > dWorkspace( nImgWidth*sizeof(Gpu::LeastSquaresSystem<float,6>), nImgHeight );
    Gpu::Image<float4, Gpu::TargetDevice, Gpu::Manage> dDebug(nImgWidth, nImgHeight);

    Gpu::Image<unsigned char, Gpu::TargetDevice, Gpu::Manage> dPrevImg(nImgWidth, nImgHeight);
    Gpu::Image<float4, Gpu::TargetDevice, Gpu::Manage> dPrevVbo(nImgWidth, nImgHeight);

    pangolin::ActivateDrawImage<unsigned char> DrawLeftImg(dLeftImg,GL_LUMINANCE8,true,true);
    pangolin::ActivateDrawImage<float4> DrawDebugImg(dDebug,GL_RGBA32F_ARB,true,true);
    pangolin::ActivateDrawImage<float> DrawDisparity(dDisp,GL_LUMINANCE32F_ARB,true,true);

    guiContainer[0].SetDrawFunction(boost::ref(DrawLeftImg));
    guiContainer[1].SetDrawFunction(boost::ref(DrawDebugImg));
    guiContainer[2].SetDrawFunction(boost::ref(DrawDisparity));


    // Pose variables
    Sophus::SE3 T_wp;
    Sophus::SE3 T_pc;

    // Start camera with robot identity
    Eigen::Matrix3d RDFvision;
    RDFvision << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::Matrix3d RDFrobot;
    RDFrobot << 0, 1, 0, 0, 0, 1, 1, 0, 0;
    Eigen::Matrix4d T_vr = Eigen::Matrix4d::Identity( );
    Eigen::Matrix4d T_rv = Eigen::Matrix4d::Identity( );
    T_vr.block < 3, 3 > (0, 0) = RDFvision.transpose( ) * RDFrobot;
    T_rv.block < 3, 3 > (0, 0) = RDFrobot.transpose( ) * RDFvision;
    T_wp = Sophus::SE3(/*T_wr*/ Eigen::Matrix4d::Identity() * T_rv);

    // gui control variables
    bool guiRunning = false;
    bool guiSetPrevious = true;
    bool guiGo = false;

    // keyboard callbacks
    pangolin::RegisterKeyPressCallback(' ',[&guiRunning](){ guiRunning = !guiRunning; });
pangolin::RegisterKeyPressCallback('p',[&guiSetPrevious](){ guiSetPrevious = true; });
pangolin::RegisterKeyPressCallback(pango::PANGO_SPECIAL + GLUT_KEY_RIGHT,[&guiGo](){ guiGo = true; });

while( !pangolin::ShouldQuit() ) {

    if( guiRunning || pangolin::Pushed(guiGo)) {
        // capture an image
        pCam->Capture( vImages );
    }

    // upload images
    dLeftImg.MemcpyFromHost(vImages[0].Image.data,nImgWidth);
    dRightImg.MemcpyFromHost(vImages[1].Image.data,nImgWidth);

    // Update our pose
    Eigen::Matrix<double,3,4> KTpc = CamModel.K() * T_pc.matrix3x4();
    Gpu::LeastSquaresSystem<float,6> LSS = Gpu::PoseRefinementFromDepthmap(dLeftImg,dPrevImg,dPrevVbo,KTpc,fNormC,dWorkspace,dDebug);
    Eigen::Matrix<double,6,6> LHS = LSS.JTJ;
    Eigen::Vector6d RHS = LSS.JTy;
    Eigen::Vector6d X;
    Eigen::FullPivLU<Eigen::Matrix<double,6,6> > lu_JTJ(LHS);
    X = - (lu_JTJ.solve(RHS));
    T_pc = T_pc * Sophus::SE3::exp(X);

    if(pangolin::Pushed(guiSetPrevious))
    {
        // calculate disparity
        Gpu::DenseStereo( dDispInt, dLeftImg, dRightImg, nMaxDisparity, 0 );
        Gpu::ReverseCheck( dDispInt, dLeftImg, dRightImg);
        Gpu::DenseStereoSubpixelRefine( dDisp, dDispInt, dLeftImg, dRightImg );
        Gpu::MedianFilterRejectNegative9x9( dDisp, dDisp, 50 );
        Gpu::FilterDispGrad(dDisp, dDisp, 2.0);
        Eigen::Matrix3d K = CamModel.K();
        Eigen::Matrix4d RightCamPose = CamModel.GetPose();

        // Store Previous image / vbo
        Gpu::DisparityImageToVbo( dPrevVbo, dDisp, RightCamPose( 1, 3 ), K( 0, 0 ), K( 1, 1 ), K( 0, 2 ), K( 1, 2 ) );
        dPrevImg.CopyFrom(dLeftImg);

        // Update 'previous' position to current position
        T_wp = T_wp * T_pc;
        T_pc = Sophus::SE3();

        // Update VBO for display
        {
            pango::CudaScopedMappedPtr var( cbo );
            Gpu::Image< uchar4 >       dCbo( (uchar4*)*var, nImgWidth, nImgHeight );
            Gpu::ConvertImage< uchar4, unsigned char >( dCbo, dPrevImg );
        }

        {
            pango::CudaScopedMappedPtr var( vbo );
            Gpu::Image< float4 >       dVbo( (float4*)*var, nImgWidth, nImgHeight );
            dVbo.CopyFrom(dPrevVbo);
        }

        // Normalise for display
        nppiDivC_32f_C1IR(nMaxDisparity,dDisp.ptr,dDisp.pitch,dDisp.Size());
    }

    ///--------------------------------------------------------------------------
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glColor4f( 1, 1, 1, 1);

    glVBO.SetPose(T_wp.matrix());
    glCurPose.SetPose( (T_wp * T_pc).matrix() );

    pangolin::FinishGlutFrame();
}

return 0;
}
