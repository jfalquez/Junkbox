
#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>
#include <SceneGraph/SimCam.h>
#include <boost/thread.hpp>
#include <Mvlpp/Mvl.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <CVars/CVar.h>
#include "CVarHelpers.h"
#include "LinearSystem.h"
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
vector < sg::ImageView* > glJacobians;


// Reference Camera Controls
Eigen::Vector6d g_dRefVel = Eigen::Vector6d::Zero();

// Global Vars
unsigned int  g_nImgWidth  = 512;
unsigned int  g_nImgHeight = 512;
volatile bool g_bLocalize  = false;
volatile bool g_bRendered  = false;
volatile bool g_bPlaying   = false;
volatile bool g_bContinue  = false;

// //////////////////////////////////////////////////////////////////////////////
void _Pause( )
{
	g_bPlaying = g_bPlaying ? false : true;
	if( g_bPlaying ) {
		g_bContinue = true;
	}
}

// //////////////////////////////////////////////////////////////////////////////
void _Continue( )
{
	g_bContinue = g_bContinue ? false : true;
}

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
            // print initial poses
            std::cout << "Reference Pose: " << g_dRefPose.transpose() << std::endl;
            std::cout << "Initial Virtual Pose: " << g_dVirtPose.transpose() << std::endl;

            // capture reference image
            Eigen::Matrix<unsigned char, 1, Eigen::Dynamic> RefImg;

            RefImg.resize( g_nImgHeight * g_nImgWidth );
            RefCam.CaptureGrey( RefImg.data() );

            // initialize system of equations
            ESM.Init( RefImg, &VirtCam, false );

            // hard limit of iterations so we don't loop forever
            int nMaxIters = 0;

            // this variable holds the estimated transform
            Eigen::Matrix4d dTrv = Eigen::Matrix4d::Identity();

            // this variable holds the delta update solution
            Eigen::Matrix4d dTdelta;

            // keep track of errors
            double NewError;
            double PrevError = ESM.Error();

            // keep track of time
            double dTs = mvl::Tic();

            // store initial pose
            Eigen::Matrix4d dInitialVirtPose = VirtCam.GetPose();
			std::cout << "Initial Virtual Pose in Vision Frame: " << mvl::T2Cart( dInitialVirtPose ).transpose() << endl;

            while( (nMaxIters < g_nMaxIterations) && g_bLocalize ) {
                std::cout << "////////////////////////////////////////////////////////////////////////////////"
                          << std::endl;

                // increment counter
                nMaxIters++;

				// solve system
                double dTi = mvl::Tic();

                dTdelta = ESM.Solve(1, glJacobians);

                std::cout << "Solving took: " << mvl::Toc( dTi ) << std::endl;

                // show solution
                std::cout << "Delta Pose is: " << mvl::T2Cart( dTdelta ).transpose() << std::endl;

                // update Trv
                ESM.ApplyUpdate();

                dTrv = dTrv * mvl::TInv( dTdelta );

                // update camera position
				Eigen::Matrix4d NewPose;
				NewPose = dInitialVirtPose * mvl::TInv( dTrv );
				VirtCam.SetPose( NewPose );
				std::cout << "New Pose in Vision Frame: " << mvl::T2Cart(NewPose).transpose() << endl;
				g_dVirtPose = mvl::T2Cart( VirtCam.GetPose() );
				std::cout << "New Pose in Robotics Frame: " << g_dVirtPose.transpose() << endl;

                // get error
                NewError = ESM.Error();

                std::cout << "New Virtual Pose is: " << g_dVirtPose.transpose() << std::endl;
                std::cout << "Error is: " << NewError << std::endl;

                // if error change is too small, break
                if( fabs(PrevError - NewError) < 1e-2 ) {
                    break;
                }

				PrevError = NewError;

				if( g_bPlaying == false ) {
					while( g_bContinue == false ) {}
					g_bContinue = false;
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
    pango::CreateGlutWindowAndBind( "Dense Pose Refinement", 1280, 640 );
    sg::GLSceneGraph::ApplyPreferredGlSettings();

    // Scenegraph to hold GLObjects and relative transformations
    sg::GLSceneGraph glGraph;

    // set up mesh
//    sg::GLMesh glMesh;

    string sMesh = cl.follow( "CityBlock.obj", 1, "-mesh" );

    try
    {
//        glMesh.Init( sMesh );
//        glMesh.SetPerceptable( true );
//        glGraph.AddChild( &glMesh );

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
    RefCam.Init( &glGraph, mvl::Cart2T( g_dRefPose ), K, g_nImgWidth, g_nImgHeight, sg::eSimCamLuminance );
    VirtCam.Init( &glGraph, mvl::Cart2T( g_dVirtPose ), K, g_nImgWidth, g_nImgHeight,
                  sg::eSimCamDepth | sg::eSimCamLuminance );

    // Define Camera Render Object (for view / scene browsing)
    pango::OpenGlRenderState glState( pango::ProjectionMatrix( 640, 480, 420, 420, 320, 240, 0.1, 1000 ),
                                      pango::ModelViewLookAt( -6, 0, -30, 1, 0, 0, pango::AxisNegZ ) );

    // Pangolin abstracts the OpenGL viewport as a View.
    // Here we get a reference to the default 'base' view.
    pango::View& glBaseView = pango::DisplayBase();

    // We define a new view which will reside within the container.
    pango::View glView;

    // We set the views location on screen and add a handler which will
    // let user input update the model_view matrix (stacks3d) and feed through
    // to our scenegraph
    glView.SetBounds( 1.0 / 3.0, 1.0, 0.0, 3.0 / 4.0, 640.0f / 480.0f );
    glView.SetHandler( new sg::HandlerSceneGraph( glGraph, glState, pango::AxisNegZ ) );
    glView.SetDrawFunction( sg::ActivateDrawFunctor( glGraph, glState ) );

    // display images
    sg::ImageView glRefImg( true, true );
    glRefImg.SetBounds( 2.0 / 3.0, 1.0, 3.0 / 4.0, 1.0, (double)g_nImgWidth / g_nImgHeight );

    sg::ImageView glVirtImg( true, true );
    glVirtImg.SetBounds( 1.0 / 3.0, 2.0 / 3.0, 3.0 / 4.0, 1.0, (double)g_nImgWidth / g_nImgHeight );

    sg::ImageView glErrorImg( true, false );
    glErrorImg.SetBounds( 0.0, 1.0 / 3.0, 3.0 / 4.0, 1.0, (double)g_nImgWidth / g_nImgHeight );

    // Add our views as children to the base container.
    glBaseView.AddDisplay( glView );
    glBaseView.AddDisplay( glRefImg );
    glBaseView.AddDisplay( glVirtImg );
    glBaseView.AddDisplay( glErrorImg );


	// Jacobian Images
	sg::ImageView g_JImgX;
	sg::ImageView g_JImgY;
	sg::ImageView g_JImgZ;
	sg::ImageView g_JImgP;
	sg::ImageView g_JImgQ;
	sg::ImageView g_JImgR;
	g_JImgX.SetBounds( 0.0, 1.0 / 3.0, 0.0, (1.0 / 6.0) * (3.0 / 4.0), (double)g_nImgWidth / g_nImgHeight );
	g_JImgY.SetBounds( 0.0, 1.0 / 3.0, (1.0 / 6.0) * (3.0 / 4.0), (2.0 / 6.0) * (3.0 / 4.0), (double)g_nImgWidth / g_nImgHeight );
	g_JImgZ.SetBounds( 0.0, 1.0 / 3.0, (2.0 / 6.0) * (3.0 / 4.0), (3.0 / 6.0) * (3.0 / 4.0), (double)g_nImgWidth / g_nImgHeight );
	g_JImgP.SetBounds( 0.0, 1.0 / 3.0, (3.0 / 6.0) * (3.0 / 4.0), (4.0 / 6.0) * (3.0 / 4.0), (double)g_nImgWidth / g_nImgHeight );
	g_JImgQ.SetBounds( 0.0, 1.0 / 3.0, (4.0 / 6.0) * (3.0 / 4.0), (5.0 / 6.0) * (3.0 / 4.0), (double)g_nImgWidth / g_nImgHeight );
	g_JImgR.SetBounds( 0.0, 1.0 / 3.0, (5.0 / 6.0) * (3.0 / 4.0), 3.0 / 4.0, (double)g_nImgWidth / g_nImgHeight );
    glBaseView.AddDisplay( g_JImgX );
    glBaseView.AddDisplay( g_JImgY );
    glBaseView.AddDisplay( g_JImgZ );
    glBaseView.AddDisplay( g_JImgP );
    glBaseView.AddDisplay( g_JImgQ );
    glBaseView.AddDisplay( g_JImgR );
	glJacobians.push_back( &g_JImgX );
	glJacobians.push_back( &g_JImgY );
	glJacobians.push_back( &g_JImgZ );
	glJacobians.push_back( &g_JImgP );
	glJacobians.push_back( &g_JImgQ );
	glJacobians.push_back( &g_JImgR );


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
    pango::RegisterKeyPressCallback( pango::PANGO_SPECIAL + GLUT_KEY_DOWN, boost::bind( _Pause ) );
    pango::RegisterKeyPressCallback( pango::PANGO_SPECIAL + GLUT_KEY_RIGHT, boost::bind( _Continue ) );

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
        glView.Activate( glState );

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

        // Pause for 1/60th of a second.
        usleep( 1E6 / 60 );
    }

    return 0;
}

// TODO: Fix coordinate system
// TODO: Write a ground truth loop.