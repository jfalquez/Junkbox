
#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>
#include <SceneGraph/SimCam.h>
#include <boost/thread.hpp>
#include <Mvlpp/Mvl.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <CVars/CVar.h>
#include "CVarHelpers.h"
#include "LinearSystemRU.h"
#include "GLImgPlane.h"

using namespace std;

namespace sg =SceneGraph;
namespace pango =pangolin;

/**************************************************************************************************
 *
 * VARIABLES
 *
 **************************************************************************************************/

// Global CVars
bool&            g_bShowFrustum   = CVarUtils::CreateCVar( "Cam.ShowFrustum", true, "Show cameras viewing frustum." );
unsigned int&    g_nMaxIterations = CVarUtils::CreateCVar( "ESM.MaxIterations", 500u, "Max number of iterations." );
Eigen::Vector6d& g_dRefPose       = CVarUtils::CreateCVar( "Cam.Pose.Ref", Eigen::Vector6d( Eigen::Vector6d::Zero() ),
                                  "Reference camera's pose." );
Eigen::Vector6d& g_dVirtPose = CVarUtils::CreateCVar( "Cam.Pose.Virt", Eigen::Vector6d( Eigen::Vector6d::Zero() ),
                                   "Virtual camera's pose." );

// Cameras
sg::GLSimCam RefCam;     // reference camera we move
sg::GLSimCam VirtCam;    // virtual camera which calculates transformation

// Reference Camera Controls
Eigen::Vector6d g_dRefVel = Eigen::Vector6d::Zero();

// Global Vars
unsigned int  g_nImgWidth  = 512;
unsigned int  g_nImgHeight = 512;
volatile bool g_bLocalize  = false;
volatile bool g_bRendered  = false;

// //////////////////////////////////////////////////////////////////////////////
void _MoveCamera(
        unsigned int DF,
        float        X
        )
{
    g_dRefVel( DF ) += X;
}

// //////////////////////////////////////////////////////////////////////////////
void _StopCamera()
{
    g_dRefVel.setZero();
}

// //////////////////////////////////////////////////////////////////////////////
void _ResetCamera()
{
    g_bLocalize = false;
    g_dVirtPose = g_dRefPose;

    VirtCam.SetPose( mvl::Cart2T( g_dVirtPose ) );
}

// //////////////////////////////////////////////////////////////////////////////
void _StartLclzr()
{
    g_bLocalize = true;
}

// //////////////////////////////////////////////////////////////////////////////
void _FlipImg(
        Eigen::Matrix<unsigned char, 1, Eigen::Dynamic>& vImg    // < Input/Output: Img buffer
        )
{
    Eigen::Matrix<unsigned char, 1, Eigen::Dynamic> tmp;

    tmp = vImg;

    unsigned int nImgHeight = RefCam.ImageHeight();
    unsigned int nImgWidth  = RefCam.ImageWidth();
    unsigned int Idx;

    for( int ii = 0; ii < nImgHeight; ii++ ) {
        for( int jj = 0; jj < nImgWidth; jj++ ) {
            Idx                       = (nImgHeight - ii - 1) * nImgWidth + jj;
            vImg[ii * nImgWidth + jj] = tmp[Idx];
        }
    }
}

// //////////////////////////////////////////////////////////////////////////////
void UpdateCameras()
{
    Eigen::Matrix4d T;

    // move camera by user's input
    T          = mvl::Cart2T( g_dRefPose );
    T          = T * mvl::Cart2T( g_dRefVel );
    g_dRefPose = mvl::T2Cart( T );

    RefCam.SetPose( mvl::Cart2T( g_dRefPose ) );
    RefCam.RenderToTexture();    // will render to texture, then copy texture to CPU memory
    VirtCam.RenderToTexture();

    g_bRendered = true;
}

// ///////////////////////////////////////////////////////////////////////////////////////
void Localizer()
{
    LinearSystem ESM;

    while( 1 ) {
        if( g_bLocalize ) {
            Eigen::Vector6d dInitialVirtPose = g_dVirtPose;

            // print initial poses
            std::cout << "Reference Pose: " << g_dRefPose.transpose() << std::endl;
            std::cout << "Initial Virtual Pose: " << g_dVirtPose.transpose() << std::endl;

            //
            // ------------------------------------- Coarse Localization
            //
            // capture reference image
            Eigen::Matrix<unsigned char, 1, Eigen::Dynamic> RefImg;

            RefImg.resize( g_nImgHeight * g_nImgWidth );
            RefCam.CaptureGrey( RefImg.data() );
            _FlipImg( RefImg );

            // initialize system of equations
            ESM.Init( RefImg, &VirtCam, true );

            // count number of iterations so we don't loop forever
            unsigned int nMaxIters = 0;

            // this variable holds the estimated transform
            Eigen::Matrix4d dTrv = Eigen::Matrix4d::Identity();

            // this variable holds the delta update solution
            Eigen::Matrix4d dTdelta;

            // keep track of time
            double dTs = mvl::Tic();

            //
            // ------------------------------------- Refine Localization
            //
            std::cout << std::endl;
            std::cout << "======================== REFINING ========================" << std::endl;
            std::cout << std::endl;

            // capture reference image
            RefImg.resize( g_nImgHeight * g_nImgWidth );
            RefCam.CaptureGrey( RefImg.data() );
            _FlipImg( RefImg );

            // initialize system of equations
            ESM.Init( RefImg, &VirtCam );

            // hard limit of iterations so we don't loop forever
            nMaxIters = 0;

            // this variable holds the estimated transform
            dTrv = Eigen::Matrix4d::Identity();

            // reset initial pose
            dInitialVirtPose = g_dVirtPose;

            while( (nMaxIters < g_nMaxIterations) && g_bLocalize ) {
                std::cout << "////////////////////////////////////////////////////////////////////////////////"
                          << std::endl;

                // increment counter
                nMaxIters++;

                // solve system
                double dTi = mvl::Tic();

                dTdelta = ESM.Solve();

                std::cout << "Solving took: " << mvl::Toc( dTi ) << std::endl;

                // show solution
                std::cout << "Delta Pose is: " << mvl::T2Cart( dTdelta ).transpose() << std::endl;

                // update Trv
                dTrv = dTrv * mvl::TInv( dTdelta );

                // update camera position
                g_dVirtPose = dInitialVirtPose - mvl::T2Cart( dTrv );

                VirtCam.SetPose( mvl::Cart2T( g_dVirtPose ) );

	            // wait for GUI loop to render in new position
		        g_bRendered = false;

			    while( g_bRendered == false ) {}

				// update image date in LinearSystem
				ESM.SnapVirtualCam();

                std::cout << "New Virtual Pose is: " << g_dVirtPose.transpose() << std::endl;
                std::cout << "Error is: " << ESM.Error() << std::endl;

                // if error change is too small, break
                if( ESM.Error() < 1e-2 ) {
                    break;
                }
            }

            dTs = mvl::TocMS( dTs );

            std::cout << std::endl << "Reference Pose: " << g_dRefPose.transpose() << std::endl;
            std::cout << "Final Estimated Pose: " << g_dVirtPose.transpose() << std::endl;
            std::cout << "Pose Difference: " << (g_dRefPose - g_dVirtPose).transpose() << std::endl;
            std::cout << "Time: " << dTs << " ms." << std::endl;

            g_bLocalize = false;
        }
    }
}

// ///////////////////////////////////////////////////////////////////////////////////////
int main(
        int    argc,
        char** argv
        )
{
    // parse arguments
    GetPot cl( argc, argv );

    // Create OpenGL window in single line thanks to GLUT
    pango::CreateGlutWindowAndBind( "Dense Pose Refinement", 640 * 3, 640 );
    sg::GLSceneGraph::ApplyPreferredGlSettings();

    // Scenegraph to hold GLObjects and relative transformations
    sg::GLSceneGraph glGraph;

    // set up mesh
    sg::GLMesh glMesh;

    string sMesh = cl.follow( "CityBlock.obj", 1, "-mesh" );

    try
    {
        glMesh.Init( sMesh );
        glMesh.SetPosition( 0, 0, -0.15 );
        glMesh.SetPerceptable( true );

        // glGraph.AddChild( &glMesh );

        cout << "Mesh '" << sMesh << "' loaded." << endl;
    }
    catch( exception e )
    {
        cerr << "Cannot load mesh. Check file exists." << endl;

        exit( 0 );
    }

    // load image plane
    cv::Mat Img;

    Img = cv::imread( "antoine.jpg", 0 );

    cv::transpose( Img, Img );

    GLImgPlane glImgPlane;

    // initialize image plane
    glImgPlane.SetImage( Img.data, Img.cols, Img.rows, GL_LUMINANCE, GL_UNSIGNED_BYTE );

    Eigen::Vector6d BasePose;

    BasePose << 80, -68.1, 102.4, 0, 0, 0;
    glImgPlane.SetBaseFrame( BasePose );
    glGraph.AddChild( &glImgPlane );

    // Define grid object
    // sg::GLGrid glGrid(50,2.0, true);
    // glGraph.AddChild(&glGrid);
    //
    // prepare K matrix
    Eigen::Matrix3d K;    // computer vision K matrix

    K << g_nImgWidth, 0, g_nImgWidth / 2, 0, g_nImgHeight, g_nImgHeight / 2, 0, 0, 1;

    // initialize cameras
    g_dRefPose << 0, -7.5, -1.5, 0, 0, 0;
    g_dVirtPose << 0, -7.5, -1.5, 0, 0, 0;
    RefCam.Init( &glGraph, mvl::Cart2T( g_dRefPose ), K, g_nImgWidth, g_nImgHeight, sg::eSimCamLuminance );
    VirtCam.Init( &glGraph, mvl::Cart2T( g_dVirtPose ), K, g_nImgWidth, g_nImgHeight,
                  sg::eSimCamDepth | sg::eSimCamLuminance );

    // Define Camera Render Object (for view / scene browsing)
    pango::OpenGlRenderState glState( pango::ProjectionMatrix( 640, 480, 420, 420, 320, 240, 0.1, 1000 ),
                                      pango::ModelViewLookAt( -30, 5, -30, 20, -10, 0, pango::AxisNegZ ) );

    // Pangolin abstracts the OpenGL viewport as a View.
    // Here we get a reference to the default 'base' view.
    pango::View& glBaseView = pango::DisplayBase();

    // We define a new view which will reside within the container.
    pango::View glView;

    // We set the views location on screen and add a handler which will
    // let user input update the model_view matrix (stacks3d) and feed through
    // to our scenegraph
    glView.SetBounds( 0.0, 1.0, 0.0, 3.0 / 4.0, 640.0f / 480.0f );
    glView.SetHandler( new sg::HandlerSceneGraph( glGraph, glState, pango::AxisNegZ ) );
    glView.SetDrawFunction( sg::ActivateDrawFunctor( glGraph, glState ) );

    // display images
    sg::ImageView glRefImg( false, true );

    glRefImg.SetBounds( 0.66, 1.0, 3.0 / 4.0, 1.0, (double)g_nImgWidth / g_nImgHeight );

    sg::ImageView glVirtImg( false, true );

    glVirtImg.SetBounds( 0.33, 0.66, 3.0 / 4.0, 1.0, (double)g_nImgWidth / g_nImgHeight );

    sg::ImageView glErrorImg( false, false );

    glErrorImg.SetBounds( 0.0, 0.33, 3.0 / 4.0, 1.0, (double)g_nImgWidth / g_nImgHeight );

    // Add our views as children to the base container.
    glBaseView.AddDisplay( glView );
    glBaseView.AddDisplay( glRefImg );
    glBaseView.AddDisplay( glVirtImg );
    glBaseView.AddDisplay( glErrorImg );

    // launch ESM thread
    boost::thread Lclzr_Thread( Localizer );

    // register key callbacks
    pango::RegisterKeyPressCallback( 'e', boost::bind( _MoveCamera, 0, 0.01 ) );
    pango::RegisterKeyPressCallback( 'q', boost::bind( _MoveCamera, 0, -0.01 ) );
    pango::RegisterKeyPressCallback( 'a', boost::bind( _MoveCamera, 1, -0.01 ) );
    pango::RegisterKeyPressCallback( 'd', boost::bind( _MoveCamera, 1, 0.01 ) );
    pango::RegisterKeyPressCallback( 'w', boost::bind( _MoveCamera, 2, -0.01 ) );
    pango::RegisterKeyPressCallback( 's', boost::bind( _MoveCamera, 2, 0.01 ) );
    pango::RegisterKeyPressCallback( 'u', boost::bind( _MoveCamera, 3, -0.005 ) );
    pango::RegisterKeyPressCallback( 'o', boost::bind( _MoveCamera, 3, 0.005 ) );
    pango::RegisterKeyPressCallback( 'i', boost::bind( _MoveCamera, 4, 0.005 ) );
    pango::RegisterKeyPressCallback( 'k', boost::bind( _MoveCamera, 4, -0.005 ) );
    pango::RegisterKeyPressCallback( 'j', boost::bind( _MoveCamera, 5, -0.005 ) );
    pango::RegisterKeyPressCallback( 'l', boost::bind( _MoveCamera, 5, 0.005 ) );
    pango::RegisterKeyPressCallback( ' ', boost::bind( _StopCamera ) );
    pango::RegisterKeyPressCallback( 'r', boost::bind( _ResetCamera ) );
    pango::RegisterKeyPressCallback( 't', boost::bind( _StartLclzr ) );

    // buffer for images
    Eigen::Matrix<unsigned char, 1, Eigen::Dynamic> vRefImg;
    Eigen::Matrix<unsigned char, 1, Eigen::Dynamic> vVirtImg;
    Eigen::Matrix<unsigned char, 1, Eigen::Dynamic> vErrorImg;

    vRefImg.resize( g_nImgWidth * g_nImgHeight );
    vVirtImg.resize( g_nImgWidth * g_nImgHeight );

    // Default hooks for exiting (Esc) and fullscreen (tab).
    while( !pango::ShouldQuit() ) {
        // Clear whole screen
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

        // pre-render stuff
        UpdateCameras();

        // render cameras
        glView.ActivateScissorAndClear( glState );

        if( g_bShowFrustum ) {
            // show the camera
            RefCam.DrawCamera();
            VirtCam.DrawCamera();
        }

        // show reference images
        if( RefCam.CaptureGrey( vRefImg.data() ) ) {
            glRefImg.SetImage( vRefImg.data(), g_nImgWidth, g_nImgHeight, GL_INTENSITY, GL_LUMINANCE,
                               GL_UNSIGNED_BYTE );
        }

        // show virtual image
        if( VirtCam.CaptureGrey( vVirtImg.data() ) ) {
            glVirtImg.SetImage( vVirtImg.data(), g_nImgWidth, g_nImgHeight, GL_INTENSITY, GL_LUMINANCE,
                                GL_UNSIGNED_BYTE );
        }

        // show error image
        vErrorImg = vRefImg - vVirtImg;

        glErrorImg.SetImage( vErrorImg.data(), g_nImgWidth, g_nImgHeight, GL_INTENSITY, GL_LUMINANCE,
                             GL_UNSIGNED_BYTE );

        // Swap frames and Process Events
        pango::FinishGlutFrame();

        // pause for 1/60th of a second.
		// go full speed if localizing
		if( g_bLocalize == false ) {
			usleep( 1E6 / 60 );
		}
    }

    return 0;
}

// TODO: Fix coordinate system
// TODO: Add imageview's for renders and error
// TODO: Use the greyscale SimCam
// TODO: Write a ground truth loop.