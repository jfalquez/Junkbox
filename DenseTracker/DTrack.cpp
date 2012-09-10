#include <pangolin/pangolin.h>
#include <pangolin/glcuda.h>
#include <sophus/se3.h>
#include <SceneGraph/SceneGraph.h>
#include <SceneGraph/SimCam.h>
#include <kangaroo/kangaroo.h>
#include <kangaroo/../applications/common/CameraModelPyramid.h>
#include <RPG/Utils/ImageWrapper.h>
//#include <boost/thread.hpp>
//#include <Mvlpp/Mvl.h>
#include <CVars/CVar.h>
#include "ParseArgs.h"
#include "GLPath.h"
#include "CVarHelpers.h"

using namespace std;

namespace sg =SceneGraph;
namespace pango =pangolin;

const int PYR_LEVELS = 5;


/**************************************************************************************************
*
* VARIABLES
*
**************************************************************************************************/
bool&         g_bShowFrustum   = CVarUtils::CreateCVar( "Gui.ShowFrustum", true, "Show cameras viewing frustum." );
unsigned int& g_nPoseDisplay   = CVarUtils::CreateCVar( "Gui.PoseDisplay", 5u,
                                   "Number of poses to display (0 displays all)." );
unsigned int& g_nMaxDisparity = CVarUtils::CreateCVar( "Gpu.MaxDisparity", 20u,
                                    "Maximum disparity for depth generation." );
Eigen::Matrix<int,1,Eigen::Dynamic>& g_vPyrCycles = CVarUtils::CreateCVar( "Tracker.PyrCycles", Eigen::Matrix<int,1,Eigen::Dynamic>(), "Number of cycles per pyramid level." );
Eigen::Matrix<int,1,Eigen::Dynamic>& g_vPyrFullMask = CVarUtils::CreateCVar( "Tracker.PyrFullMask", Eigen::Matrix<int,1,Eigen::Dynamic>(), "Set 1 for full estimate, 0 for rotation only estimates." );

/// Camera Model Information
mvl::CameraModel* g_CamModel;
#define g_mTvl ( Eigen::Matrix4d::Identity() )
#define g_mTvr ( g_CamModel->GetPose() )
#define g_nImgWidth ( g_CamModel->Width() )
#define g_nImgHeight ( g_CamModel->Height() )
#define g_nImgSize ( g_nImgWidth * g_nImgHeight )
#define g_K ( g_CamModel->K() )

/// Image Containers
vector< rpg::ImageWrapper > g_vCapturedImg;
vector< rpg::ImageWrapper > g_vPrevCapturedImg;
vector< cv::Mat >           g_vVirtualImg;
vector< cv::Mat >           g_vErrorImg;

/// Sim Cameras
Eigen::Matrix4d g_mCamPose;
sg::GLSimCam    glLeftCam;
sg::GLSimCam    glRightCam;
#define g_mTwv ( g_mCamPose )
#define g_mTwl ( g_mCamPose )
#define g_mTwr ( g_mTwl * g_mTvr )

/////////////////////////////////////////////////////////////////////////////////
inline void SnapVirtualImages()
{
    glLeftCam.RenderToTexture();    // will render to texture, then copy texture to CPU memory
    glRightCam.RenderToTexture();

    // populate matrices
    if( glLeftCam.CaptureGrey( g_vVirtualImg[0].data ) == false ) {
        cerr << "Error capturing left image." << endl;
    }

    if( glLeftCam.CaptureDepth( g_vVirtualImg[1].data ) == false ) {
        cerr << "Error capturing left image." << endl;
    }

    if( glRightCam.CaptureGrey( g_vVirtualImg[2].data ) == false ) {
        cerr << "Error capturing right image." << endl;
    }

    if( glRightCam.CaptureDepth( g_vVirtualImg[3].data ) == false ) {
        cerr << "Error capturing right image." << endl;
    }

    // flip images
//    cv::flip( g_vVirtualImg[0], g_vVirtualImg[0], 0 );
//    cv::flip( g_vVirtualImg[1], g_vVirtualImg[1], 0 );
//    cv::flip( g_vVirtualImg[2], g_vVirtualImg[2], 0 );
//    cv::flip( g_vVirtualImg[3], g_vVirtualImg[3], 0 );
}

/////////////////////////////////////////////////////////////////////////////////
inline void NormalizeDepth(
        float*              Depth,
        const unsigned int& Size
        )
{
    // find max depth
    float MaxDepth = 0;
    for( unsigned int ii = 0; ii < Size; ii++ ) {
        if( MaxDepth < Depth[ii] ) {
            MaxDepth = Depth[ii];
        }
    }

    if( MaxDepth == 0 ) {
        return;
    }

    // normalize
    for( unsigned int ii = 0; ii < Size; ii++ ) {
        Depth[ii] = Depth[ii] / MaxDepth;
    }
}

/**************************************************************************************************
*
* MAIN
*
**************************************************************************************************/
int main(
        int    argc,
        char** argv
        )
{
    // parse arguments
    GetPot cl( argc, argv );

    // parse parameters
    CameraDevice* pCam = ParseArgs( argc, argv );

    // read camera model file
    std::string sCamModFileName = pCam->GetProperty( "CamModelFile", "rcmod.xml" );
    g_CamModel = new mvl::CameraModel( sCamModFileName );
    CameraModelPyramid CamModelPyr( sCamModFileName );
    CamModelPyr.PopulatePyramid( PYR_LEVELS );

    // initialize camera pose
    g_mCamPose = Eigen::Matrix4d::Identity();

    // Create OpenGL window in single line thanks to GLUT
    pango::CreateGlutWindowAndBind( "Dense Tracker", 1280, 768 );
    sg::GLSceneGraph::ApplyPreferredGlSettings();
    cudaGLSetGLDevice( 0 );

    // Scenegraph to hold GLObjects and relative transformations
    sg::GLSceneGraph glGraph;

    // Define Camera Render Object (for view / scene browsing)
    pango::OpenGlRenderState glState( pango::ProjectionMatrix( 640, 480, 420, 420, 320, 240, 0.1, 1000 ),
                                      pango::ModelViewLookAt( -6, 0, -10, 1, 0, 0, pango::AxisNegZ ) );

    // Pangolin abstracts the OpenGL viewport as a View.
    pango::View& glBaseView = pango::DisplayBase();

    // We define a new view which will reside within the container.
    pango::View glView3D;

    // We set the views location on screen and add a handler
    glView3D.SetBounds( 0.0, 1.0, 0.45, 1.0, 640.0f / 480.0f );
    glView3D.SetHandler( new sg::HandlerSceneGraph( glGraph, glState, pango::AxisNegZ ) );
    glView3D.SetDrawFunction( sg::ActivateDrawFunctor( glGraph, glState ) );

    // draw grid on 3D window
    sg::GLGrid glGrid;
    glGraph.AddChild( &glGrid );

    // initialize cameras
    glLeftCam.Init( &glGraph, g_mTwl, g_K, g_nImgWidth, g_nImgHeight, sg::eSimCamDepth | sg::eSimCamLuminance );
    glRightCam.Init( &glGraph, g_mTwr, g_K, g_nImgWidth, g_nImgHeight, sg::eSimCamDepth | sg::eSimCamLuminance );

    // prepare virtual images
    g_vVirtualImg.resize( 4 );

    g_vVirtualImg[0] = cv::Mat( g_nImgHeight, g_nImgWidth, CV_8UC1 );     // left greyscale
    g_vVirtualImg[1] = cv::Mat( g_nImgHeight, g_nImgWidth, CV_32FC1 );    // left depth
    g_vVirtualImg[2] = cv::Mat( g_nImgHeight, g_nImgWidth, CV_8UC1 );     // right greyscale
    g_vVirtualImg[3] = cv::Mat( g_nImgHeight, g_nImgWidth, CV_32FC1 );    // right depth

    g_vErrorImg.resize( 2 );    // only calculate greyscale image error

    // capture initial virtual images
    SnapVirtualImages();

    // initialize vbo's
    pango::GlBufferCudaPtr vbo( pango::GlArrayBuffer, g_nImgWidth, g_nImgHeight, GL_FLOAT, 4,
                                cudaGraphicsMapFlagsWriteDiscard, GL_STREAM_DRAW );
    pango::GlBufferCudaPtr cbo( pango::GlArrayBuffer, g_nImgWidth, g_nImgHeight, GL_UNSIGNED_BYTE, 4,
                                cudaGraphicsMapFlagsWriteDiscard, GL_STREAM_DRAW );
    pango::GlBufferCudaPtr ibo( pango::GlElementArrayBuffer, g_nImgWidth, g_nImgHeight, GL_UNSIGNED_INT, 2 );
    sg::GLVbo glVBO( &vbo, &ibo, &cbo );

    // Generate Index Buffer Object for rendering mesh
    {
        pango::CudaScopedMappedPtr var( ibo );
        Gpu::Image< uint2 >        dIbo( (uint2*)*var, g_nImgWidth, g_nImgHeight );
        Gpu::GenerateTriangleStripIndexBuffer( dIbo );
    }
    glGraph.AddChild( &glVBO );

    // add path to 3D window
    GLPath     glPath;
    glGraph.AddChild( &glPath );

    // display captured images
    sg::ImageView glImgLeft( true, true );
    glImgLeft.SetBounds( 2.0 / 3.0, 1.0, 0.0, 0.15, (double)g_nImgWidth / g_nImgHeight );
    sg::ImageView glImgRight( true, true );
    glImgRight.SetBounds( 2.0 / 3.0, 1.0, 0.15, 0.3, (double)g_nImgWidth / g_nImgHeight );
    sg::ImageView glDepth( true, false );
    glDepth.SetBounds( 2.0 / 3.0, 1.0, 0.3, 0.45, (double)g_nImgWidth / g_nImgHeight );

    // display virtual images (ie. generated by SimCam)
    sg::ImageView glVImgLeft( true, true );
    glVImgLeft.SetBounds( 1.0 / 3.0, 2.0 / 3.0, 0.0, 0.15, (double)g_nImgWidth / g_nImgHeight );
    sg::ImageView glVImgRight( true, true );
    glVImgRight.SetBounds( 1.0 / 3.0, 2.0 / 3.0, 0.15, 0.3, (double)g_nImgWidth / g_nImgHeight );
    sg::ImageView glVDepth( true, false );
    glVDepth.SetBounds( 1.0 / 3.0, 2.0 / 3.0, 0.3, 0.45, (double)g_nImgWidth / g_nImgHeight );

    // display error images
    sg::ImageView glEImgLeft( true, true );
    glEImgLeft.SetBounds( 0.0, 1.0 / 3.0, 0.0, 0.15, (double)g_nImgWidth / g_nImgHeight );
    sg::ImageView glEImgRight( true, true );
    glEImgRight.SetBounds( 0.0, 1.0 / 3.0, 0.15, 0.3, (double)g_nImgWidth / g_nImgHeight );

    // add our views as children to the base container.
    glBaseView.AddDisplay( glView3D );
    glBaseView.AddDisplay( glImgLeft );
    glBaseView.AddDisplay( glImgRight );
    glBaseView.AddDisplay( glDepth );
    glBaseView.AddDisplay( glVImgLeft );
    glBaseView.AddDisplay( glVImgRight );
    glBaseView.AddDisplay( glVDepth );
    glBaseView.AddDisplay( glEImgLeft );
    glBaseView.AddDisplay( glEImgRight );

    // launch tracker thread
//    boost::thread TrackerThread( _TrackerLoop, pCam );

    // prepare GPU images
    Gpu::Pyramid<unsigned char, PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage> d_LeftPyr( g_nImgWidth, g_nImgHeight );
    Gpu::Pyramid<unsigned char, PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage> d_RightPyr( g_nImgWidth, g_nImgHeight );
    Gpu::Pyramid<unsigned char, PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage> d_VirtPyr( g_nImgWidth, g_nImgHeight );
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage > d_BlurTmp1( g_nImgWidth, g_nImgHeight );
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage > d_BlurTmp2( g_nImgWidth, g_nImgHeight );
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage > d_DispInt( g_nImgWidth, g_nImgHeight );
    Gpu::Pyramid< float, PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage > d_DispPyr( g_nImgWidth, g_nImgHeight );
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage > d_Workspace( g_nImgWidth * sizeof(Gpu::LeastSquaresSystem<float,6>), g_nImgHeight );
    Gpu::Image< float4, Gpu::TargetDevice, Gpu::Manage > d_Debug( g_nImgWidth, g_nImgHeight );

//    Gpu::Pyramid< float, PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage > dDispPyrC( nImgWidth, nImgHeight );

    // Pose variables
    Eigen::Matrix4d Trl = Eigen::Matrix4d::Identity();

    // Start camera with robot identity
    Eigen::Matrix4d T_vis2rob = Eigen::Matrix4d::Identity( );
    Eigen::Matrix4d T_rob2vis = Eigen::Matrix4d::Identity( );
    {
        Eigen::Matrix3d RDFvision;
        RDFvision << 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1;
        Eigen::Matrix3d RDFrobot;
        RDFrobot << 0, 1, 0,
                    0, 0, 1,
                    1, 0, 0;
        T_vis2rob.block < 3, 3 > (0, 0) = RDFvision.transpose( ) * RDFrobot;
        T_rob2vis.block < 3, 3 > (0, 0) = RDFrobot.transpose( ) * RDFvision;
    }
//    g_mTwv = /*T_wr*/ Eigen::Matrix4d::Identity() * T_rob2vis;
//    glPath.PushPose( T_rob2vis );

    // initialize number of iterations to perform at each pyramid level
    // level 0 is finest (ie. biggest image)
    g_vPyrCycles.resize( PYR_LEVELS );
    g_vPyrCycles << 1, 2, 3, 4, 5;
    g_vPyrFullMask.resize( PYR_LEVELS );
    g_vPyrFullMask << 1, 1, 1, 1, 0;


    // side panel
    pangolin::Var<unsigned int> nBlur("ui.Blur",1,0,5);
    pangolin::Var<int> nMaxDisparity("ui.MaxDisp",16,0,100);
    pangolin::Var<int> nMedianFilter("ui.MedianFilter",50,0,100);
    pangolin::Var<float> fNormC("ui.Norm C",50,0,100);
    pangolin::Var<float> fSqErr("ui.Mean Sq Error");
    pangolin::Var<float> fKeyThreshold("ui.Keyframe Threshold",0.75,0,1);

    // gui control variables
    bool guiStep = true;
    bool guiRunning = false;
    bool guiSetKeyframe = true;

    // synch variables
    bool bVboInit = false;
    bool bVirtualDirty = true;  // force a capture of virtual images

    // register key callbacks
    pangolin::RegisterKeyPressCallback(' ',[&guiRunning](){ guiRunning = !guiRunning; });
    pangolin::RegisterKeyPressCallback('k',[&guiSetKeyframe](){ guiSetKeyframe = true; });
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + GLUT_KEY_RIGHT,[&guiStep](){ guiStep = true; });

    while( !pango::ShouldQuit() ) {

        // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Capture
        //
        if( guiRunning || pangolin::Pushed(guiStep)) {
            // capture an image
            pCam->Capture( g_vCapturedImg );

            // handy alias to avoid long names
            const cv::Mat& LeftImg = g_vCapturedImg[0].Image;
            const cv::Mat& RightImg = g_vCapturedImg[1].Image;

            // upload images
            d_LeftPyr[0].MemcpyFromHost( LeftImg.data, g_nImgWidth );
            d_RightPyr[0].MemcpyFromHost( RightImg.data, g_nImgWidth );

            // blur bottom image
            for (int ii = 0; ii < nBlur; ++ii) {
                Gpu::Blur( d_LeftPyr[0], d_BlurTmp1 );
                Gpu::Blur( d_RightPyr[0], d_BlurTmp1 );
            }

            // reduce and blur rest of pyramid
            Gpu::BlurReduce<unsigned char, PYR_LEVELS, unsigned int>(d_LeftPyr, d_BlurTmp1, d_BlurTmp2);
            Gpu::BlurReduce<unsigned char, PYR_LEVELS, unsigned int>(d_RightPyr, d_BlurTmp1, d_BlurTmp2 );

            // TODO: these do NOT show any blur.. we have to bring them down from GPU
            // if we want to see blur on GUI

            // show left image
            glImgLeft.SetImage( LeftImg.data, g_nImgWidth, g_nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_UNSIGNED_BYTE );

            // show right image
            glImgRight.SetImage( RightImg.data, g_nImgWidth, g_nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_UNSIGNED_BYTE );
        }


        // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Capture Virtual Images
        //
        if( bVirtualDirty ) {
            // render to texture
            SnapVirtualImages();

            // upload image
            d_VirtPyr[0].MemcpyFromHost( g_vVirtualImg[0].data, g_nImgWidth );

            // upload depthmap
            d_DispPyr[0].MemcpyFromHost( g_vVirtualImg[1].data, g_nImgWidth*4 );
            Gpu::BoxReduce< float, PYR_LEVELS, float >( d_DispPyr );

            // blur bottom image
            /*
            for (int ii = 0; ii < nBlur; ++ii) {
                Gpu::Blur( d_VirtPyr[0], d_BlurTmp1 );
            }
            */

            // reduce and blur rest of pyramid
            Gpu::BlurReduce<unsigned char, PYR_LEVELS, unsigned int>( d_VirtPyr, d_BlurTmp1, d_BlurTmp2 );

            // update image holders
            glVImgLeft.SetImage( g_vVirtualImg[0].data, g_nImgWidth, g_nImgHeight, GL_INTENSITY, GL_LUMINANCE,
                                 GL_UNSIGNED_BYTE );
            glVImgRight.SetImage( g_vVirtualImg[2].data, g_nImgWidth, g_nImgHeight, GL_INTENSITY, GL_LUMINANCE,
                                  GL_UNSIGNED_BYTE );

            // normalize left depth map
            cv::Mat DepthMap = g_vVirtualImg[1].clone();
            NormalizeDepth( (float*)DepthMap.data, g_nImgWidth * g_nImgHeight );
            glVDepth.SetImage( DepthMap.data, g_nImgWidth, g_nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_FLOAT );

            // calculate new image error
            g_vErrorImg[0] = g_vCapturedImg[0].Image - g_vVirtualImg[0];
            g_vErrorImg[1] = g_vCapturedImg[1].Image - g_vVirtualImg[2];

            // update image holders
            glEImgLeft.SetImage( g_vErrorImg[0].data, g_nImgWidth, g_nImgHeight, GL_INTENSITY, GL_LUMINANCE,
                                 GL_UNSIGNED_BYTE );
            glEImgRight.SetImage( g_vErrorImg[1].data, g_nImgWidth, g_nImgHeight, GL_INTENSITY, GL_LUMINANCE,
                                  GL_UNSIGNED_BYTE );

            bVirtualDirty = false;
        }


        // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Localize
        //
        unsigned int nObs;
        if( bVboInit ) {
            for( int PyrLvl = PYR_LEVELS-1; PyrLvl >= 0; PyrLvl-- ) {
                for( int ii = 0; ii < g_vPyrCycles[PyrLvl]; ii++ ) {
                    const unsigned w = (int)g_nImgWidth >> PyrLvl;
                    const unsigned h = (int)g_nImgHeight >> PyrLvl;

                    Eigen::Matrix3d Kpyr = CamModelPyr.K(PyrLvl);
                    Sophus::SE3 sTrl = Sophus::SE3( Trl );
                    Eigen::Matrix<double,3,4> KTlr = Kpyr * sTrl.inverse().matrix3x4();
                    const float fBaseline = (1 << PyrLvl) * CamModelPyr.GetPose()( 1, 3 );

                    // build system
                    Gpu::LeastSquaresSystem<float,6> LSS = Gpu::PoseRefinementFromDisparity( d_LeftPyr[PyrLvl],d_VirtPyr[PyrLvl],
                                                                                            d_DispPyr[PyrLvl],Kpyr,KTlr,fNormC,fBaseline,
                                                                                            d_Workspace,d_Debug.SubImage(w,h) );
                    Eigen::Matrix<double,6,6> LHS = LSS.JTJ;
                    Eigen::Vector6d RHS = LSS.JTy;

                    // solve system
                    Eigen::Vector6d X;

                    // check if we are solving only for rotation, or full estimate
                    if( g_vPyrFullMask(PyrLvl) != 0 ) {
                        Eigen::FullPivLU<Eigen::Matrix<double,6,6> > lu_JTJ(LHS);

                        // check degenerate system
                        if( lu_JTJ.rank() < 6 ) {
                            cerr << "warning(@L" << PyrLvl << "I" << ii << ") LS trashed. " << "Rank: " << lu_JTJ.rank() << endl;
                            continue;
                        }

                        X = - (lu_JTJ.solve(RHS));
                    } else {
                        // extract rotation information only
                        Eigen::Matrix<double,3,3> rLHS = LHS.block<3,3>(3,3);
                        Eigen::Vector3d rRHS = RHS.tail(3);
                        Eigen::FullPivLU<Eigen::Matrix<double,3,3> > lu_JTJ(rLHS);

                        // check degenerate system
                        if( lu_JTJ.rank() < 3 ) {
                            cerr << "warning(@L" << PyrLvl << "I" << ii << ") LS trashed. " << "Rank: " << lu_JTJ.rank() << endl;
                            continue;
                        }

                        Eigen::Vector3d rX;
                        rX = - (lu_JTJ.solve(rRHS));

                        // pack solution
                        X.setZero();
                        X.tail(3) = rX;
                    }

                    // if we have too few observations, discard estimate
                    if( (float)LSS.obs / (float)( w * h ) < fMinPts ) {
                        cerr << "warning(@L" << PyrLvl << "I" << ii << ") LS trashed. " << "Too few pixels!" << endl;
                        continue;
                    }

                    // everything seems fine... apply update
                    Trl = (Trl.inverse() * Sophus::SE3::exp(X).matrix()).inverse();
                    fSqErr =  LSS.sqErr / LSS.obs;

                    // only store nObs of last level
                    if( PyrLvl == 0 ) {
                        nObs = LSS.obs;
                    }

                    // mark virtual cameras as dirty to force re-rendering
                    bVirtualDirty = true;
                }
            }
        }

        // don't know where to put this shit
        glPath.PushPose( Trl );
//        Twv = Twv * Trl;
        Trl = Eigen::Matrix4d::Identity();

        // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Keyframe & VBO Generation
        //
        if( pangolin::Pushed(guiSetKeyframe) || (float)nObs / (float)( g_nImgWidth * g_nImgHeight ) < fKeyThreshold )
        {
            // create a colored buffer object
            {
                pangolin::CudaScopedMappedPtr var( cbo );
                Gpu::Image< uchar4 >          dCbo( (uchar4*)*var, g_nImgWidth, g_nImgHeight );
                Gpu::ConvertImage< uchar4, unsigned char >( dCbo, d_LeftPyr[0] );
            }

            // check if depth map is given to us by camera...
            if( g_vCapturedImg.size() == 3 ) {
                cv::Mat                              DepthMap = g_vCapturedImg[2].Image.clone();
                Gpu::Image< float, Gpu::TargetHost > hDepth( (float*)DepthMap.data, g_nImgWidth, g_nImgHeight );
                d_DispPyr[0].CopyFrom( hDepth );
                Gpu::BoxReduce< float, PYR_LEVELS, float >( d_DispPyr );

                {
                    pango::CudaScopedMappedPtr var( vbo );
                    Gpu::Image< float4 >       dVbo( (float4*)*var, g_nImgWidth, g_nImgHeight );
                    Gpu::DepthToVbo( dVbo, d_DispPyr[0], g_K( 0, 0 ), g_K( 1, 1 ), g_K( 0, 2 ), g_K( 1, 2 ) );
                }

                // show depth image (normalize first)
                NormalizeDepth( (float*)DepthMap.data, g_nImgWidth * g_nImgHeight );
                glDepth.SetImage( DepthMap.data, g_nImgWidth, g_nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_FLOAT );
            } else {
                // ... if not, calculate depth.
                Gpu::DenseStereo( d_DispInt, d_LeftPyr[0], d_RightPyr[0], nMaxDisparity, 0 );
                Gpu::ReverseCheck( d_DispInt, d_LeftPyr[0], d_RightPyr[0]);
                Gpu::DenseStereoSubpixelRefine( d_DispPyr[0], d_DispInt, d_LeftPyr[0], d_RightPyr[0] );
                Gpu::MedianFilterRejectNegative9x9( d_DispPyr[0], d_DispPyr[0], nMedianFilter );
                Gpu::FilterDispGrad( d_DispPyr[0], d_DispPyr[0], 2.0 );
                Gpu::BoxReduce< float, PYR_LEVELS, float >( d_DispPyr );

                {
                    pango::CudaScopedMappedPtr var( vbo );
                    Gpu::Image< float4 >       dVbo( (float4*)*var, g_nImgWidth, g_nImgHeight );
                    Gpu::DisparityImageToVbo( dVbo, d_DispPyr[0], g_mTvr( 1, 3 ), g_K( 0, 0 ), g_K( 1, 1 ), g_K( 0, 2 ), g_K( 1, 2 ) );
                }

                // copy depth map from GPU memory in order to display on GUI
                Gpu::Image< float, Gpu::TargetHost, Gpu::Manage > hDepth( g_nImgWidth, g_nImgHeight );
                hDepth.CopyFrom( d_DispPyr[0] );
                NormalizeDepth( hDepth.ptr, g_nImgWidth * g_nImgHeight );
                glDepth.SetImage( hDepth.ptr, g_nImgWidth, g_nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_FLOAT );
            }

            // set base pose for VBO relative to current pose
            // so convert pose to vision frame
            Eigen::Matrix4d M;
            M << 0, 0, 1, 0,
                 1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 0, 1;
            M = g_mTwv * M;
            glVBO.SetPose( M );

            bVboInit = true;
        }


        ///------------------------------------------------------------------------------------------------------------
        /// DRAW STUFF
        ///------------------------------------------------------------------------------------------------------------

        // update stuff
        glLeftCam.SetPoseRobot( g_mTwl );
        glRightCam.SetPoseRobot( g_mTwr );

        // clear whole screen
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

        // draw the cameras
        glView3D.Activate( glState );

        if( g_bShowFrustum ) {
            glLeftCam.DrawCamera();
            glRightCam.DrawCamera();
        }

        // swap frames and process events
        pango::FinishGlutFrame();

        // 60 Hz refresh rate
        usleep( 1E6 / 60 );
//        sleep(10);
    }

    return 0;
}

// TODO:
