
#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>
#include <SceneGraph/SimCam.h>
#include <boost/bind.hpp>
#include <Mvlpp/Mvl.h>
#include <CVars/CVar.h>
#include "CVarHelpers.h"
#include "ImageExporter.h"

using namespace std;
namespace sg =SceneGraph;
namespace pango =pangolin;

// //////////////////////////////////////////////////////////////////////////////
const char USAGE[] =
    "Usage:     MeshLogger -idev <input> <options>\n" "\n" "where input device can be: FileReader Bumblebee2 etc\n"
    "\n" "Input Specific Options:\n" "   FileReader:      -lfile <regular expression for left image channel>\n"
    "                    -rfile <regular expression for right image channel>\n"
    "                    -sdir  <directory where source images are located [default '.']>\n"
    "                    -sf    <start frame [default 0]>\n" "\n"
    "General Options:    -lcmod <left camera model xml file>\n"
    "                    -rcmod <right camera model xml file>\n"
    "                    -gt <ground truth file> [not required]\n" "\n" "Example:\n"
    "gslam  -idev FileReader  -lcmod lcmod.xml  -rcmod rcmod.xml  -lfile \"left.*pgm\"  -rfile \"right.*pgm\"\n";

/**************************************************************************************************
 *
 * VARIABLES
 *
 **************************************************************************************************/

// Global CVars
bool&            g_bShowFrustum = CVarUtils::CreateCVar( "Cam.ShowFrustum", true, "Show cameras viewing frustum." );
Eigen::Vector6d& g_dCamPose     = CVarUtils::CreateCVar( "Cam.Pose", Eigen::Vector6d( Eigen::Vector6d::Zero() ),
                                  "Camera's pose." );

// Cameras
sg::GLSimCam glCamLeft;     // reference camera we move
sg::GLSimCam glCamRight;    // reference camera we move

// Reference Camera Controls
Eigen::Vector6d g_dCamVel = Eigen::Vector6d::Zero();

// Global Variables
bool			g_bCapture;
Eigen::Vector6d g_dBaseline;
unsigned int    g_nImgWidth;
unsigned int    g_nImgHeight;

// //////////////////////////////////////////////////////////////////////////////
void _ResetCamera()
{
    // set camera from initial view
}

// //////////////////////////////////////////////////////////////////////////////
void _MoveCamera(
        unsigned int DF,
        float        X
        )
{
    g_dCamVel( DF ) += X;
}

// //////////////////////////////////////////////////////////////////////////////
void _StopCamera()
{
    g_dCamVel.setZero();
}

// //////////////////////////////////////////////////////////////////////////////
void _Capture()
{
	g_bCapture = true;
}

// //////////////////////////////////////////////////////////////////////////////
inline void UpdateCameras()
{
    Eigen::Matrix4d T;

    // move camera by user's input
    T          = mvl::Cart2T( g_dCamPose );
    T          = T * mvl::Cart2T( g_dCamVel );
    g_dCamPose = mvl::T2Cart( T );

    glCamLeft.SetPose( mvl::Cart2T( g_dCamPose - g_dBaseline / 2 ) );
    glCamRight.SetPose( mvl::Cart2T( g_dCamPose + g_dBaseline / 2 ) );

    glCamLeft.RenderToTexture();    // will render to texture, then copy texture to CPU memory
    glCamRight.RenderToTexture();    // will render to texture, then copy texture to CPU memory
}

// //////////////////////////////////////////////////////////////////////////////
inline void NormalizeDepth(
        float*       Depth,
        unsigned int Size
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

// //////////////////////////////////////////////////////////////////////////////
struct DrawLeftImage
{
    void operator ()(
            pangolin::View&
            )
    {
        glPushAttrib( GL_ENABLE_BIT );
        glDisable( GL_LIGHTING );

        // Activate viewport
        // this->Activate();
        // glColor3f(1,1,1);
        // Load orthographic projection matrix to match image
        glMatrixMode( GL_PROJECTION );

        // m_ortho.Load();
        // Reset ModelView matrix
        glMatrixMode( GL_MODELVIEW );
        glLoadIdentity();

        // Render texture
        glBindTexture( GL_TEXTURE_2D, glCamLeft.GreyTexture() );
        glEnable( GL_TEXTURE_2D );
        glBegin( GL_QUADS );
        glTexCoord2f( 0, 0 );
        glVertex2d( -0.5, -0.5 );
        glTexCoord2f( 1, 0 );
        glVertex2d( glCamLeft.ImageWidth() - 0.5, -0.5 );
        glTexCoord2f( 1, 1 );
        glVertex2d( glCamLeft.ImageWidth() - 0.5, glCamLeft.ImageHeight() - 0.5 );
        glTexCoord2f( 0, 1 );
        glVertex2d( -0.5, glCamLeft.ImageHeight() - 0.5 );
        glEnd();
        glDisable( GL_TEXTURE_2D );

        // Call base View implementation
        // pangolin::View::Render();
        glPopAttrib();

        cout << "DONE!" << endl;
    }
};


// //////////////////////////////////////////////////////////////////////////////
bool LoadPoseFile( const string sFileName, vector< Eigen::Vector6d >& vPoses )
{
        ifstream        pFile;
        Eigen::Vector6d Pose;

        pFile.open( sFileName.c_str() );

        if( pFile.is_open() ) {
            cout << "MeshLogger: File opened..." << endl;

            while( !pFile.eof() ) {
                pFile >> Pose( 0 ) >> Pose( 1 ) >> Pose( 2 ) >> Pose( 3 ) >> Pose( 4 ) >> Pose( 5 );

                vPoses.push_back( Pose );
            }
        } else {
            cerr << "MeshLogger: Error opening file!" << endl;
			return false;
        }

        pFile.close();

        cout << "MeshLogger: File closed." << endl;
        cout << "MeshLogger: " << vPoses.size() << " poses read." << endl;
        return true;

    return false;
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
    if( argc < 1 ) {
        cout << USAGE << endl;

        exit( 0 );
    }

    // parse arguments
    GetPot cl( argc, argv );

    if( cl.search( 3, "--help", "-help", "-h" ) ) {
        cout << USAGE << endl;

        exit( 0 );
    }

	bool            bCaptureDepth;
    bCaptureDepth = cl.search( "-depth" );

	bool            bCaptureSeq;
    bCaptureSeq = cl.search( "-seq" );

    string sMesh             = cl.follow( "CityBlock.obj", 1, "-mesh" );
    string sInputName         = cl.follow( "", 1, "-file" );
    string sLeftCameraModel  = cl.follow( "lcmod.xml", 1, "-lcmod" );
    string sRightCameraModel = cl.follow( "rcmod.xml", 1, "-rcmod" );

    // get camera model files
    mvl::CameraModel LeftCamModel( sLeftCameraModel );
    mvl::CameraModel RightCamModel( sRightCameraModel );

    // get some camera parameters
    Eigen::Matrix3d K = LeftCamModel.K();

    g_nImgWidth  = LeftCamModel.Width();
    g_nImgHeight = LeftCamModel.Height();

    // for the baseline we assume the left is the dominant camera
    Eigen::Matrix4d LeftCamPose  = LeftCamModel.GetPose();
    Eigen::Matrix4d RightCamPose = RightCamModel.GetPose();

    g_dBaseline << 0, RightCamPose( 1, 3 ) - LeftCamPose( 1, 3 ), 0, 0, 0, 0;

    // Create OpenGL window in single line thanks to GLUT
    pango::CreateGlutWindowAndBind( "MeshLogger", 640 * 3, 640 );
    sg::GLSceneGraph::ApplyPreferredGlSettings();

    // Scenegraph to hold GLObjects and relative transformations
    sg::GLSceneGraph glGraph;

    // set up mesh
    sg::GLMesh glMesh;

    try
    {
        glMesh.Init( sMesh );
        glMesh.SetPosition( 0, 0, -0.15 );
        glMesh.SetPerceptable( true );
        glGraph.AddChild( &glMesh );

        cout << "MeshLogger: Mesh '" << sMesh << "' loaded." << endl;
    }
    catch( exception e )
    {
        cerr << "MeshLogger: Cannot load mesh. Check file exists." << endl;

        exit( 0 );
    }

    // define grid object
    sg::GLGrid glGrid( 50, 2.0, true );

    glGrid.SetPose( 0, 1, 0, 0, 0, 0 );
    glGraph.AddChild( &glGrid );

	// load file of poses (if provided)
	unsigned int PoseIdx = 0;
	vector< Eigen::Vector6d > vPoses;
	if( sInputName.empty() == false ) {
		if( LoadPoseFile( sInputName, vPoses ) == false ) {
			exit(0);
		}
		// set camera to initial position
		g_dCamPose = vPoses[0];
		PoseIdx = 1;
	}

    // initialize cameras
    if( bCaptureDepth ) {
        glCamLeft.Init( &glGraph, mvl::Cart2T( g_dCamPose - g_dBaseline / 2 ), K, g_nImgWidth, g_nImgHeight,
                        sg::eSimCamLuminance | sg::eSimCamDepth );
    } else {
        glCamLeft.Init( &glGraph, mvl::Cart2T( g_dCamPose - g_dBaseline / 2 ), K, g_nImgWidth, g_nImgHeight,
                        sg::eSimCamLuminance );
    }

    glCamRight.Init( &glGraph, mvl::Cart2T( g_dCamPose + g_dBaseline / 2 ), K, g_nImgWidth, g_nImgHeight,
                     sg::eSimCamLuminance );

    // Define Camera Render Object (for view / scene browsing)
    pango::OpenGlRenderState glState( pango::ProjectionMatrix( 640, 480, 420, 420, 320, 240, 0.1, 1000 ),
                                      pango::ModelViewLookAt( -6, 0, -30, 1, 0, 0, pango::AxisNegZ ) );

    // Pangolin abstracts the OpenGL viewport as a View.
    // Here we get a reference to the default 'base' view.
    pango::View& glBaseView = pango::DisplayBase();

    // We define a new view which will reside within the container.
    pango::View glView3D;

    // We set the views location on screen and add a handler which will
    // let user input update the model_view matrix (stacks3d) and feed through
    // to our scenegraph
    glView3D.SetBounds( 0.0, 1.0, 0.0, 3.0 / 4.0, 640.0f / 480.0f );
    glView3D.SetHandler( new sg::HandlerSceneGraph( glGraph, glState, pango::AxisNegZ ) );
    glView3D.SetDrawFunction( sg::ActivateDrawFunctor( glGraph, glState ) );

    // display images
    sg::ImageView glLeftImg( false, true );

    glLeftImg.SetBounds( 0.66, 1.0, 3.0 / 4.0, 1.0, (double)g_nImgWidth / g_nImgHeight );

    sg::ImageView glRightImg( false, true );

    glRightImg.SetBounds( 0.33, 0.66, 3.0 / 4.0, 1.0, (double)g_nImgWidth / g_nImgHeight );

    sg::ImageView glDepthImg( false, false );

    if( bCaptureDepth ) {
        glDepthImg.SetBounds( 0.0, 0.33, 3.0 / 4.0, 1.0, (double)g_nImgWidth / g_nImgHeight );
    }

    // Add our views as children to the base container.
    glBaseView.AddDisplay( glView3D );
    glBaseView.AddDisplay( glLeftImg );
    glBaseView.AddDisplay( glRightImg );

    if( bCaptureDepth ) {
        glBaseView.AddDisplay( glDepthImg );
    }

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
    pango::RegisterKeyPressCallback( 'c', boost::bind( _Capture ) );
    pango::RegisterKeyPressCallback( pango::PANGO_CTRL + 'r', boost::bind( _ResetCamera ) );

    // buffer for our images and depth map
    char* pBuffImg   = (char*)malloc( g_nImgWidth * g_nImgHeight );
    float*         pBuffDepth = (float*)malloc( g_nImgWidth * g_nImgHeight );

	// set up image exporters
	ImageExporter ieLeft;
	ImageExporter ieRight;

	if( bCaptureSeq ) {
		ieLeft.Init( "LeftSeq" );
		ieRight.Init( "RightSeq" );
	} else {
		ieLeft.Init("Left", g_nImgWidth, g_nImgHeight );
		ieRight.Init("Right", g_nImgWidth, g_nImgHeight );
	}

	// reset capture flag
	g_bCapture = false;

    // Default hooks for exiting (Esc) and fullscreen (tab).
    while( !pango::ShouldQuit() ) {
        // Clear whole screen
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

        // pre-render stuff
        UpdateCameras();

        // render cameras
        glView3D.ActivateScissorAndClear( glState );

        if( g_bShowFrustum ) {
            // show the camera
            glCamLeft.DrawCamera();
            glCamRight.DrawCamera();
        }

        // show left images
        if( glCamLeft.CaptureGrey( pBuffImg ) ) {
            glLeftImg.SetImage( pBuffImg, g_nImgWidth, g_nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_UNSIGNED_BYTE );
			if( g_bCapture ) ieLeft.Export( pBuffImg );
        }

        // show right image
        if( glCamRight.CaptureGrey( pBuffImg ) ) {
            glRightImg.SetImage( pBuffImg, g_nImgWidth, g_nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_UNSIGNED_BYTE );
			if( g_bCapture ) ieRight.Export( pBuffImg );
        }

        // show depth
        if( bCaptureDepth ) {
            if( glCamLeft.CaptureDepth( pBuffDepth ) ) {
                NormalizeDepth( pBuffDepth, g_nImgWidth * g_nImgHeight );
                glDepthImg.SetImage( pBuffDepth, g_nImgWidth, g_nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_FLOAT );
            }
        }

		// reset capture flag
		if( g_bCapture ) g_bCapture = false;

		// if we are rendering from an input file...
		// .. set capture flag and reposition camera
		if( PoseIdx != 0 ) {
			g_bCapture = true;
			g_dCamPose += vPoses[PoseIdx];
			PoseIdx++;
			if( PoseIdx >= vPoses.size() ) {
				cout << "MeshLogger: Finished rendering input file." << endl;
				exit(0);
			}
		}

        // Swap frames and Process Events
        pango::FinishGlutFrame();

        // Pause for 1/60th of a second.
        usleep( 1E6 / 60 );
    }

    return 0;
}

// TODO: Add support in ImageExporter for depth map (float images).
// TODO: Make sure video export is working properly.