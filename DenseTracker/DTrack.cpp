
#include <pangolin/pangolin.h>
#include <pangolin/glcuda.h>
#include <SceneGraph/SceneGraph.h>
#include <SceneGraph/SimCam.h>
#include <kangaroo/kangaroo.h>
#include <RPG/Utils/ImageWrapper.h>
#include <boost/thread.hpp>
#include <Mvlpp/Mvl.h>

//#include <opencv/cv.h>
//#include <opencv2/highgui/highgui.hpp>

#include <CVars/CVar.h>
#include "ParseArgs.h"
#include "GLPath.h"
//#include "LinearSystem.h"
#include "LinearSystemRU.h"

using namespace std;

namespace sg =SceneGraph;
namespace pango =pangolin;

/**************************************************************************************************
*
* VARIABLES
*
**************************************************************************************************/
unsigned int& g_nMaxIterations = CVarUtils::CreateCVar( "ESM.MaxIterations", 100u, "Max number of iterations." );
bool&         g_bShowFrustum   = CVarUtils::CreateCVar( "Gui.ShowFrustum", true, "Show cameras viewing frustum." );
unsigned int& g_nPoseDisplay   = CVarUtils::CreateCVar( "Gui.PoseDisplay", 5u,
                                   "Number of poses to display (0 displays all)." );
unsigned int& g_nMaxDisparity = CVarUtils::CreateCVar( "Gpu.MaxDisparity", 20u,
                                    "Maximum disparity for depth generation." );
unsigned int& g_nFilter33Iters = CVarUtils::CreateCVar( "Gpu.Filter.Iters3", 0u,
                                     "Number of iterations for the median 3x3 filter." );
unsigned int& g_nFilter55Iters = CVarUtils::CreateCVar( "Gpu.Filter.Iters5", 5u,
                                     "Number of iterations for the median 5x5 filter." );

/// Camera Model Information
mvl::CameraModel* g_CamModel;
#define g_mTvl ( Eigen::Matrix4d::Identity() )
#define g_mTvr ( g_CamModel->GetPose() )
#define g_nImgWidth ( g_CamModel->Width() )
#define g_nImgHeight ( g_CamModel->Height() )
#define g_nImgSize ( g_nImgWidth * g_nImgHeight )
#define K ( g_CamModel->K() )

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

/// GL Objects
GLPath     glPath;
sg::GLVbo* glVBO;

/// Synch Stuff
volatile bool g_bCaptureDirty;
volatile bool g_bVirtualDirty;

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
    cv::flip( g_vVirtualImg[0], g_vVirtualImg[0], 0 );
    cv::flip( g_vVirtualImg[1], g_vVirtualImg[1], 0 );
    cv::flip( g_vVirtualImg[2], g_vVirtualImg[2], 0 );
    cv::flip( g_vVirtualImg[3], g_vVirtualImg[3], 0 );
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

/////////////////////////////////////////////////////////////////////////////////
void _TrackerLoop(
        CameraDevice* pCam
        )
{
    // set up LinearSystem
    LinearSystem ESM;

    while( 1 ) {
        // ///////////////////////////////////////////////////////////////////////////////
        // Capture Images
        //

        // save previous images
        g_vPrevCapturedImg = g_vCapturedImg;

        // capture new images
        // but first, wait for GUI to finish copying data if needed
        while( g_bCaptureDirty ) {}

//        pCam->Capture( g_vCapturedImg );

        // set dirty bit so GUI knows it needs to update
        g_bCaptureDirty = true;

        // set base pose for VBO relative to current pose
        // so convert pose to vision frame
        Eigen::Matrix4d M;
        M << 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;

        M = g_mTwv * M;

        glVBO->SetPose( M );

		sleep(1);

        /* */

        // ///////////////////////////////////////////////////////////////////////////////
        // Optimization Loop
        //

        // initialize ESM with: K, CurImg, PrevImg, PrevDepthMap
//        ESM.Init( K, g_vCapturedImg[0].Image, g_vPrevCapturedImg[0].Image, g_vPrevCapturedImg[2].Image );
//        ESM.Init( K, g_vCapturedImg[0].Image, g_vVirtualImg[0], g_vVirtualImg[1] );
        ESM.Init( g_vCapturedImg[0].Image, &glLeftCam );

        // hard limit of iterations so we don't loop forever
        unsigned int nMaxIters = 0;

        // this variable holds the estimated transform
        Eigen::Matrix4d dTrv = Eigen::Matrix4d::Identity();

        // this variable holds the delta update solution
        Eigen::Matrix4d dTdelta;

        // keep track of errors
        double NewError;
        double PrevError = ESM.Error();

        // store initial pose
        Eigen::Matrix4d mInitialPose = g_mCamPose;

        while( nMaxIters < g_nMaxIterations ) {
            // increment counter
            nMaxIters++;

            // solve system
            double dTi = mvl::Tic();

            // solve with 1, 4 or 8 threads
            dTdelta = ESM.Solve( 8 );

//          std::cout << "Solving took: " << mvl::Toc( dTi ) << std::endl;

            // update Trv
            ESM.ApplyUpdate();

            dTrv = dTrv * mvl::TInv( dTdelta );

            // update camera position
            //g_mCamPose      = mInitialPose * mvl::TInv( dTrv );

			g_bVirtualDirty = true;

            while( g_bVirtualDirty ) {}

			// reinitalize
//	        ESM.Init( K, g_vCapturedImg[0].Image, g_vVirtualImg[0], g_vVirtualImg[1] );
			ESM.SnapVirtualCam();

            // get error
            NewError = ESM.Error();

          std::cout << "Error is: " << NewError << std::endl;
            // if error change is too small, break
            if( fabs( PrevError - NewError ) < 1e-2 ) {
                break;
			}

            PrevError = NewError;
        }

        // show solution
        std::cout << "Estimated Delta: " << mvl::T2Cart( dTrv ).transpose() << std::endl;

        glPath.PushPose( mvl::TInv( dTrv ) );

        /*  */
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

    // initialize camera pose
    g_mCamPose = Eigen::Matrix4d::Identity();

    // capture an initial image to get image params
    pCam->Capture( g_vCapturedImg );

    g_bCaptureDirty = true;

    if( g_vCapturedImg.size() == 0 ) {
        cerr << "No images found!" << endl;

        exit( 0 );
    }

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
    glLeftCam.Init( &glGraph, g_mTwl, K, g_nImgWidth, g_nImgHeight, sg::eSimCamDepth | sg::eSimCamLuminance );
    glRightCam.Init( &glGraph, g_mTwr, K, g_nImgWidth, g_nImgHeight, sg::eSimCamDepth | sg::eSimCamLuminance );

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
    glVBO = new sg::GLVbo( &vbo, &ibo, &cbo );

    // Generate Index Buffer Object for rendering mesh
    {
        pango::CudaScopedMappedPtr var( ibo );
        Gpu::Image< uint2 >        dIbo( (uint2*)*var, g_nImgWidth, g_nImgHeight );
        Gpu::GenerateTriangleStripIndexBuffer( dIbo );
    }
    glGraph.AddChild( glVBO );

    // add path to 3D window
    glPath.PushPose( Eigen::Vector6d( Eigen::Vector6d::Zero() ) );
    Eigen::Vector3d dRot;
    dRot << 0, 10, 0;

    // glPath.SetRotation( dRot );
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

    // register key callbacks
    // pango::RegisterKeyPressCallback( 't', boost::bind( _StartLclzr ) );

    // launch tracker thread
    boost::thread TrackerThread( _TrackerLoop, pCam );

    // prepare GPU images
    Gpu::Image< float, Gpu::TargetDevice, Gpu::Manage >         dDepth( g_nImgWidth, g_nImgHeight );
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage > dLeftImg( g_nImgWidth, g_nImgHeight );
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage > dRightImg( g_nImgWidth, g_nImgHeight );
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage > dDisp( g_nImgWidth, g_nImgHeight );

    // ///////////////////////////////////////////////////////////////////////////////
    // Screen Rendering
    //
    while( !pango::ShouldQuit() ) {
        // update capture data if needed
        if( g_bCaptureDirty ) {
            cv::Mat& LeftImg = g_vCapturedImg[0].Image;

            glImgLeft.SetImage( LeftImg.data, g_nImgWidth, g_nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_UNSIGNED_BYTE );

            // show right image
            cv::Mat& RightImg = g_vCapturedImg[1].Image;
            glImgRight.SetImage( RightImg.data, g_nImgWidth, g_nImgHeight, GL_INTENSITY, GL_LUMINANCE,
                                 GL_UNSIGNED_BYTE );

            Gpu::Image< unsigned char, Gpu::TargetHost > hLeftImg( LeftImg.data, g_nImgWidth, g_nImgHeight );
            dLeftImg.CopyFrom( hLeftImg );
            {
                pango::CudaScopedMappedPtr var( cbo );
                Gpu::Image< uchar4 >       dCbo( (uchar4*)*var, g_nImgWidth, g_nImgHeight );
                Gpu::ConvertImage< uchar4, unsigned char >( dCbo, dLeftImg );
            }

            // check if depth map is given to us by camera
            if( g_vCapturedImg.size() == 3 ) {
                cv::Mat                              DepthMap = g_vCapturedImg[2].Image.clone();
                Gpu::Image< float, Gpu::TargetHost > hDepth( (float*)DepthMap.data, g_nImgWidth, g_nImgHeight );
                dDepth.CopyFrom( hDepth );

                {
                    pango::CudaScopedMappedPtr var( vbo );
                    Gpu::Image< float4 >       dVbo( (float4*)*var, g_nImgWidth, g_nImgHeight );
                    Gpu::DepthToVbo( dVbo, dDepth, K( 0, 0 ), K( 1, 1 ), K( 0, 2 ), K( 1, 2 ) );
                }

                // show depth image (normalize first)
                NormalizeDepth( (float*)DepthMap.data, g_nImgWidth * g_nImgHeight );
                glDepth.SetImage( DepthMap.data, g_nImgWidth, g_nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_FLOAT );
            } else {
                Gpu::Image< unsigned char, Gpu::TargetHost > hRightImg( RightImg.data, g_nImgWidth, g_nImgHeight );
                dRightImg.CopyFrom( hRightImg );

                Gpu::DenseStereo( dDisp, dLeftImg, dRightImg, 20, 0 );
                Gpu::DenseStereoSubpixelRefine( dDepth, dDisp, dLeftImg, dRightImg );

                for( int ii = 0; ii < g_nFilter33Iters; ii++ ) {
                    Gpu::MedianFilter3x3( dDepth, dDepth );
                }

                for( int ii = 0; ii < g_nFilter55Iters; ii++ ) {
                    Gpu::MedianFilter5x5( dDepth, dDepth );
                }

                {
                    pango::CudaScopedMappedPtr var( vbo );
                    Gpu::Image< float4 >       dVbo( (float4*)*var, g_nImgWidth, g_nImgHeight );
                    Gpu::DisparityImageToVbo( dVbo, dDepth, g_mTvr( 1, 3 ), K( 0, 0 ), K( 1, 1 ), K( 0, 2 ), K( 1, 2 ) );
                }

                // copy depth map from GPU memory in order to display on GUI
                Gpu::Image< float, Gpu::TargetHost, Gpu::Manage > hDepth( g_nImgWidth, g_nImgHeight );
                hDepth.CopyFrom( dDepth );
                NormalizeDepth( hDepth.ptr, g_nImgWidth * g_nImgHeight );
                glDepth.SetImage( hDepth.ptr, g_nImgWidth, g_nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_FLOAT );
            }

            g_bCaptureDirty = false;
        }

        // update cameras poses
        glLeftCam.SetRobotPose( g_mTwl );
        glRightCam.SetRobotPose( g_mTwr );

        // update virtual images if needed
        if( g_bVirtualDirty ) {
            // render to texture
            SnapVirtualImages();

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

            g_bVirtualDirty = false;
        }

        // clear whole screen
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

        // draw the cameras
        glView3D.ActivateScissorAndClear( glState );

        if( g_bShowFrustum ) {
            glLeftCam.DrawCamera();
            glRightCam.DrawCamera();
        }

        // swap frames and process events
        pango::FinishGlutFrame();

        // 60 Hz refresh rate
//        usleep( 1E6 / 60 );
    }

    return 0;
}

// TODO: load ground truth values