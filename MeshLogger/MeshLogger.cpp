
#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>
#include <SceneGraph/SimCam.h>
#include <boost/bind.hpp>
#include <Mvlpp/Mvl.h>
#include <CVars/CVar.h>
#include "CVarHelpers.h"

namespace sg =SceneGraph;
namespace pango =pangolin;

/**************************************************************************************************
 *
 * VARIABLES
 *
 **************************************************************************************************/
#define IMG_HEIGHT 512
#define IMG_WIDTH  512

// Global CVars
bool&            g_bShowFrustum = CVarUtils::CreateCVar( "Cam.ShowFrustum", true, "Show cameras viewing frustum." );
Eigen::Vector6d& g_dCamPose     = CVarUtils::CreateCVar( "Cam.Pose", Eigen::Vector6d( Eigen::Vector6d::Zero() ),
                                  "Camera's pose (left camera is dominant camera)." );

// Cameras
sg::GLSimCam glCamLeft;     // reference camera we move
sg::GLSimCam glCamRight;    // reference camera we move

// Reference Camera Controls
Eigen::Vector6d g_dCamVel = Eigen::Vector6d::Zero();

// Global Variables
Eigen::Vector6d g_dBaseline;
unsigned int    g_nImgWidth;
unsigned int    g_nImgHeight;

// ///////////////////////////////////////////////////////////////////////////////////////
void _MoveCamera(
        unsigned int DF,
        float        X
        )
{
    g_dCamVel( DF ) += X;
}

// ///////////////////////////////////////////////////////////////////////////////////////
void _StopCamera()
{
    g_dCamVel.setZero();
}

// ///////////////////////////////////////////////////////////////////////////////////////
void UpdateCameraPose()
 {
	Eigen::Matrix4d T;
	T = mvl::Cart2T( g_dCamPose );
	T = T * mvl::Cart2T( g_dCamVel );
    g_dCamPose = mvl::T2Cart(T);

    glCamLeft.SetPose( mvl::Cart2T( g_dCamPose - g_dBaseline / 2 ) );
    glCamRight.SetPose( mvl::Cart2T( g_dCamPose + g_dBaseline / 2 ) );
    glEnable( GL_LIGHTING );
    glEnable( GL_LIGHT0 );
    glClearColor( 0.0, 0.0, 0.0, 1 );
    glCamLeft.RenderToTexture();    // will render to texture, then copy texture to CPU memory
    glCamRight.RenderToTexture();    // will render to texture, then copy texture to CPU memory
}

// ///////////////////////////////////////////////////////////////////////////////////////
void ShowCameraAndTextures(
        pango::View&              Vw,
        pango::OpenGlRenderState& St
        )
 {
    Vw.ActivateScissorAndClear( St );

    if( g_bShowFrustum ) {
        // show the camera
        glCamLeft.DrawCamera();
        glCamRight.DrawCamera();
    }

    // / show textures
    if( glCamLeft.HasGrey() ) {
//        DrawTextureAsWindowPercentage( glCamLeft.GreyTexture(), glCamLeft.ImageWidth(), glCamLeft.ImageHeight(), 0,
//                                       0.66, 0.33, 1 );
//        DrawBorderAsWindowPercentage( 0, 0.66, 0.33, 1 );
    }

    if( glCamRight.HasGrey() ) {
//        DrawTextureAsWindowPercentage( glCamRight.GreyTexture(), glCamRight.ImageWidth(), glCamRight.ImageHeight(),
//                                       0.33, 0.66, 0.66, 1 );
//        DrawBorderAsWindowPercentage( 0.33, 0.66, 0.66, 1 );
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
    std::string sMesh = cl.follow( "CityBlock.obj", 1, "-mesh" );
    sg::GLMesh  glMesh;

    try
    {
        glMesh.Init( sMesh );
        glMesh.SetPosition( 0, 0, -0.15 );
        glMesh.SetPerceptable( true );
        glGraph.AddChild( &glMesh );

        std::cout << "Mesh '" << sMesh << "' loaded." << std::endl;
    }
    catch( std::exception e )
    {
        std::cerr << "Cannot load mesh. Check file exists." << std::endl;
    }

    // Define grid object
    sg::GLGrid glGrid( 50, 2.0, true );
    glGrid.SetPose( 0, 1, 0, 0, 0, 0 );
    glGraph.AddChild( &glGrid );

    // prepare K matrix
    Eigen::Matrix3d dK;    // computer vision K matrix

    dK << IMG_WIDTH, 0, IMG_WIDTH / 2, 0, IMG_HEIGHT, IMG_HEIGHT / 2, 0, 0, 1;

    // baseline.. eventually obtain this from camera file
    g_dBaseline << 0, 0.10, 0, 0, 0, 0;

    // initialize cameras
    glCamLeft.Init( &glGraph, mvl::Cart2T( g_dCamPose - g_dBaseline / 2 ), dK, IMG_WIDTH, IMG_HEIGHT,
                    sg::eSimCamLuminance );
    glCamRight.Init( &glGraph, mvl::Cart2T( g_dCamPose + g_dBaseline / 2 ), dK, IMG_WIDTH, IMG_HEIGHT,
                     sg::eSimCamLuminance );

    // Define Camera Render Object (for view / scene browsing)
    pango::OpenGlRenderState glState( pango::ProjectionMatrix( 640, 480, 420, 420, 320, 240, 0.1, 1000 ),
                                      pango::ModelViewLookAt( -10, -10, -20, 0, 0, 0, pango::AxisNegZ ) );

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

    // Add our views as children to the base container.
    glBaseView.AddDisplay( glView );

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

    // Default hooks for exiting (Esc) and fullscreen (tab).
    while( !pango::ShouldQuit() ) {
        // Clear whole screen
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

        // pre-render stuff
        UpdateCameraPose();

        ShowCameraAndTextures( glView, glState );

        // Swap frames and Process Events
        pango::FinishGlutFrame();

        // Pause for 1/60th of a second.
        usleep( 1E6 / 60 );
    }

    return 0;
}