#include <pangolin/pangolin.h>
#include <pangolin/glcuda.h>
#include <kangaroo/kangaroo.h>
#include <kangaroo/../applications/common/ImageSelect.h>
#include <SceneGraph/SceneGraph.h>
#include <RPG/Devices.h>
#include <Mvlpp/Mvl.h>
#include <boost/bind.hpp>
#include "ParseArgs.h"

using namespace std;


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

    const unsigned int nImgWidth = vImages[1].Image.cols;
    const unsigned int nImgHeight = vImages[0].Image.rows;

    // create a GUI window
    pangolin::CreateGlutWindowAndBind("Dense Tracker",1680,1050);
    glewInit();
    cudaGLSetGLDevice(0);
    SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState glState( pangolin::ProjectionMatrix( 640, 480, 420, 420, 320, 240, 0.1, 1000 ),
                                         pangolin::ModelViewLookAt( -6, 0, -10, 1, 0, 0, pangolin::AxisNegZ ) );

    // Scenegraph to hold GLObjects and relative transformations
    SceneGraph::GLSceneGraph glGraph;

    // We define a new view which will reside within the container.
    pangolin::View glView3D;

    // We set the views location on screen and add a handler
    glView3D.SetHandler( new SceneGraph::HandlerSceneGraph( glGraph, glState, pangolin::AxisNegZ ) );
    glView3D.SetDrawFunction( SceneGraph::ActivateDrawFunctor( glGraph, glState ) );

    // create a side panel
    pangolin::CreatePanel("ui").SetBounds(0,1,0,pangolin::Attach::Pix(300));
    pangolin::Var<unsigned int> nImgIdx("ui.Image ID", 0);
    pangolin::Var<unsigned int> nBlur("ui.Blur",1,0,5);
    pangolin::Var<int> nMaxDisparity("ui.MaxDisp",16,0,100);
    pangolin::Var<int> nMedianFilter("ui.MedianFilter",50,0,100);

    // create a view container
    pangolin::View& guiContainer = pangolin::CreateDisplay()
            .SetBounds(0,1,pangolin::Attach::Pix(300),1)
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
    SceneGraph::GLGrid glGrid;
    glGraph.AddChild( &glGrid );

    // initialize vbo's
    pangolin::GlBufferCudaPtr vbo( pangolin::GlArrayBuffer, nImgWidth, nImgHeight, GL_FLOAT, 4,
                                   cudaGraphicsMapFlagsWriteDiscard, GL_STREAM_DRAW );
    pangolin::GlBufferCudaPtr cbo( pangolin::GlArrayBuffer, nImgWidth, nImgHeight, GL_UNSIGNED_BYTE, 4,
                                   cudaGraphicsMapFlagsWriteDiscard, GL_STREAM_DRAW );
    pangolin::GlBufferCudaPtr ibo( pangolin::GlElementArrayBuffer, nImgWidth, nImgHeight, GL_UNSIGNED_INT, 2 );

    // create vbo
    SceneGraph::GLVbo glVBO( &vbo, &ibo, &cbo );

    // Start VBO in robotics frame
    {
        Eigen::Matrix3d RDFvision;
        RDFvision << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        Eigen::Matrix3d RDFrobot;
        RDFrobot << 0, 1, 0, 0, 0, 1, 1, 0, 0;
        Eigen::Matrix4d T_rv = Eigen::Matrix4d::Identity( );
        T_rv.block < 3, 3 > (0, 0) = RDFrobot.transpose( ) * RDFvision;
        glVBO.SetPose ( T_rv );
    }

    // Generate Index Buffer Object for rendering mesh
    {
        pangolin::CudaScopedMappedPtr var( ibo );
        Gpu::Image< uint2 >        dIbo( (uint2*)*var, nImgWidth, nImgHeight );
        Gpu::GenerateTriangleStripIndexBuffer( dIbo );
    }
    glGraph.AddChild(&glVBO);

    // gpu variables
    Gpu::Image<unsigned char, Gpu::TargetDevice, Gpu::Manage>     dLeftImg(nImgWidth, nImgHeight);
    Gpu::Image<unsigned char, Gpu::TargetDevice, Gpu::Manage>     dRightImg(nImgWidth, nImgHeight);
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage >   dDispInt( nImgWidth, nImgHeight );
    Gpu::Image< float, Gpu::TargetDevice, Gpu::Manage >           dDispFloat( nImgWidth, nImgHeight );
    Gpu::Image< float, Gpu::TargetDevice, Gpu::Manage >           dDispFloatNorm( nImgWidth, nImgHeight );
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage >   dBlurTmp( nImgWidth, nImgHeight );

    pangolin::ActivateDrawImage<unsigned char> DrawLeftImg( dLeftImg, GL_LUMINANCE8, false, true );
    pangolin::ActivateDrawImage<unsigned char> DrawRightImg( dRightImg, GL_LUMINANCE8, false, true );
    pangolin::ActivateDrawImage<float>         DrawDisparity( dDispFloat, GL_LUMINANCE32F_ARB, false, true );

    // add images to the container
    guiContainer[0].SetDrawFunction(boost::ref(DrawLeftImg));
    guiContainer[1].SetDrawFunction(boost::ref(DrawRightImg));
    guiContainer[2].SetDrawFunction(boost::ref(DrawDisparity));

    // gui control variables
    bool guiGo = false;
    bool guiRunning = false;

    // keyboard callbacks
    pangolin::RegisterKeyPressCallback(' ',[&guiRunning](){ guiRunning = !guiRunning; });
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + GLUT_KEY_RIGHT,[&guiGo](){ guiGo = true; });

    // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Main Loop
    //
    while( !pangolin::ShouldQuit() ) {

        // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Upload and Process Images
        //
        dLeftImg.MemcpyFromHost( vImages[0].Image.data, nImgWidth );
        dRightImg.MemcpyFromHost( vImages[1].Image.data, nImgWidth );

        // add blur
        for (int ii = 0; ii < nBlur; ++ii) {
            Gpu::Blur( dLeftImg, dBlurTmp );
            Gpu::Blur( dRightImg, dBlurTmp );
        }

        // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Generate VBO
        //
        Gpu::DenseStereo( dDispInt, dLeftImg, dRightImg, (int)nMaxDisparity, (float)0, 2 );
//        Gpu::ReverseCheck( dDispInt, dLeftImg, dRightImg);
        Gpu::DenseStereoSubpixelRefine( dDispFloat, dDispInt, dLeftImg, dRightImg );
        Gpu::MedianFilterRejectNegative9x9( dDispFloat, dDispFloat, nMedianFilter );
        Gpu::FilterDispGrad( dDispFloat, dDispFloat, 2.0 );

        // Update CBO
        {
            pangolin::CudaScopedMappedPtr var( cbo );
            Gpu::Image< uchar4 >          dCbo( (uchar4*)*var, nImgWidth, nImgHeight );
            Gpu::ConvertImage< uchar4, unsigned char >( dCbo, dLeftImg );
        }

        // Update VBO
        {
            Eigen::Matrix3d K = CamModel.K();
            const float fBaseline = CamModel.GetPose()( 1, 3 );
            pangolin::CudaScopedMappedPtr var( vbo );
            Gpu::Image< float4 >          dVbo( (float4*)*var, nImgWidth, nImgHeight );
            Gpu::DisparityImageToVbo( dVbo, dDispFloat, fBaseline, K( 0, 0 ), K( 1, 1 ), K( 0, 2 ), K( 1, 2 ) );
        }

        // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Save VBO and Capture New Images
        //
        if( guiRunning || pangolin::Pushed(guiGo)) {


            // copy disparity image from GPU
            Gpu::Image< float, Gpu::TargetHost, Gpu::Manage > hDisp( nImgWidth, nImgHeight );
            hDisp.CopyFrom( dDispFloat );

            // prepare export image
            cv::Mat DispImg( nImgWidth, nImgHeight, CV_32FC1, hDisp.ptr );

            // file info
            string sFilePrefix = "disp-";
            char   Index[10];
            unsigned int Idx = nImgIdx;
            sprintf( Index, "%05d", Idx );

            // save in PDM format
            string sFileName = sFilePrefix + Index + ".pdm";
            ofstream bFile( sFileName.c_str(), ios::out | ios::binary );
            bFile << "P7" << std::endl;
            bFile << nImgWidth << " " << nImgHeight << std::endl;
            unsigned int nSize = DispImg.elemSize1() * nImgHeight * nImgWidth;
            bFile << 4294967295 << std::endl;
            bFile.write( (const char*)DispImg.data, nSize );
            bFile.close();

            // capture a new image
            pCam->Capture( vImages );
            Idx++;
            nImgIdx = Idx;
        }

        ///------------------------------------------------------------------------------------------------------------

        // update and render stuff
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
        glColor4f( 1, 1, 1, 1);

        // normalize disparity image to visualize it better
        dDispFloatNorm.CopyFrom( dDispFloat );
        nppiDivC_32f_C1IR( nMaxDisparity, dDispFloatNorm.ptr, dDispFloatNorm.pitch, dDispFloatNorm.Size() );

        pangolin::FinishGlutFrame();
    }

return 0;
}
