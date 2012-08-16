
#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>
#include <RPG/Utils/ImageWrapper.h>
#include <boost/thread.hpp>
#include <Mvlpp/Mvl.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <CVars/CVar.h>
#include "ParseArgs.h"
#include "GLPath.h"
#include "LinearSystem.h"

// #include "CVarHelpers.h"
using namespace std;

namespace sg =SceneGraph;
namespace pango =pangolin;

/**************************************************************************************************
 *
 * VARIABLES
 *
 **************************************************************************************************/
unsigned int&    g_nMaxIterations = CVarUtils::CreateCVar( "ESM.MaxIterations", 300u, "Max number of iterations." );
unsigned int&    g_nPoseDisplay = CVarUtils::CreateCVar( "Gui.PoseDisplay", 3u, "Number of poses to display (0 displays all)." );

// intrinsics matrix
Eigen::Matrix3d K;

// image containers
vector<rpg::ImageWrapper> g_vImages;
vector<rpg::ImageWrapper> g_vPrevImages;

// image dimensions
unsigned int g_nImgHeight;
unsigned int g_nImgWidth;

// position of our axis
Eigen::Vector6d g_vCurPose;
Eigen::Vector6d g_vPrevPose;

// glPath object
GLPath glPath;


// //////////////////////////////////////////////////////////////////////////////
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

// //////////////////////////////////////////////////////////////////////////////
void _TrackerLoop( CameraDevice* pCam, sg::ImageView* pImgLeft, sg::ImageView* pImgRight, sg::ImageView* pImgDepth )
{
	// set up LinearSystem
	LinearSystem ESM;

	while(1) {

		// //////////////////////////////////////////////////////////////////////////////
		// Capture Images
		//

		// set previous data
		g_vPrevPose = g_vCurPose;
		g_vPrevImages = g_vImages;

		// capture new images
        pCam->Capture( g_vImages );

		// update images in GUI
        // show left image
		cv::Mat LeftImg;
		LeftImg = g_vImages[0].Image.clone();

        pImgLeft->SetImage( g_vImages[0].Image.data, g_nImgWidth, g_nImgHeight, GL_INTENSITY, GL_LUMINANCE,
                            GL_UNSIGNED_BYTE );

        // show right image
		cv::Mat RightImg;
		RightImg = g_vImages[1].Image.clone();
        pImgRight->SetImage( g_vImages[1].Image.data, g_nImgWidth, g_nImgHeight, GL_INTENSITY, GL_LUMINANCE,
                             GL_UNSIGNED_BYTE );

        // show depth image (normalize first)
		cv::Mat DepthMap;
		DepthMap = g_vImages[2].Image.clone();
 		NormalizeDepth( (float*)DepthMap.data, g_nImgWidth * g_nImgHeight );
		pImgDepth->SetImage( DepthMap.data, g_nImgWidth, g_nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_FLOAT );

		// //////////////////////////////////////////////////////////////////////////////
		// Optimization Loop
		//

		// initialize ESM with: K, CurImg, PrevImg, PrevDepthMap
		ESM.Init( K, g_vImages[0].Image, g_vPrevImages[0].Image, g_vPrevImages[2].Image );

		// hard limit of iterations so we don't loop forever
		unsigned int nMaxIters = 0;

		// this variable holds the estimated transform
		Eigen::Matrix4d dTrv = Eigen::Matrix4d::Identity();

		// this variable holds the delta update solution
		Eigen::Matrix4d dTdelta;

		// keep track of errors
		double NewError;
		double PrevError = ESM.Error();

		while( nMaxIters < g_nMaxIterations ) {

			// increment counter
			nMaxIters++;

			// solve system
			double dTi = mvl::Tic();

			// solve with 1, 4 or 8 threads
			dTdelta = ESM.Solve(8);

//			std::cout << "Solving took: " << mvl::Toc( dTi ) << std::endl;

			// update Trv
			ESM.ApplyUpdate();

			dTrv = dTrv * mvl::TInv( dTdelta );

			// update camera position
			g_vCurPose = g_vPrevPose - mvl::T2Cart( dTrv );

			// get error
			NewError = ESM.Error();
//			std::cout << "Error is: " << NewError << std::endl;

			// if error change is too small, break
			if( fabs(PrevError - NewError) < 1e-2 ) {
				break;
			}

			PrevError = NewError;
		}
		// show solution
		std::cout << "Estimated Delta: " << mvl::T2Cart( dTrv ).transpose() << std::endl;

		glPath.PushPose( mvl::TInv(dTrv) );
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
	std::string sCamModFileName = pCam->GetProperty( "CamModelFile", "lcmod.xml" );
	mvl::CameraModel CamModel( sCamModFileName );
	K = CamModel.K();

    // capture an initial image to get image params
    pCam->Capture( g_vImages );

    unsigned int nNumImgs;

    nNumImgs = g_vImages.size();

    if( nNumImgs == 0 ) {
        cerr << "No images found!" << endl;

        exit( 0 );
    }

    g_nImgHeight = g_vImages[0].Image.rows;
    g_nImgWidth  = g_vImages[0].Image.cols;

    // Create OpenGL window in single line thanks to GLUT
    pango::CreateGlutWindowAndBind( "Dense Tracker", 640, 768 );
    sg::GLSceneGraph::ApplyPreferredGlSettings();

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
    glView3D.SetBounds( 0.0, 3.0 / 4.0, 0.0, 1.0, 640.0f / 480.0f );
    glView3D.SetHandler( new sg::HandlerSceneGraph( glGraph, glState, pango::AxisNegZ ) );
    glView3D.SetDrawFunction( sg::ActivateDrawFunctor( glGraph, glState ) );

    // draw grid on 3D window
    sg::GLGrid glGrid;

    glGraph.AddChild( &glGrid );

	// add path to 3D window
	glPath.PushPose( Eigen::Vector6d( Eigen::Vector6d::Zero() ) );
	Eigen::Vector3d dRot;
	dRot << 0, 10, 0;
//	glPath.SetRotation( dRot );
    glGraph.AddChild( &glPath );

    // display images
    sg::ImageView glImgLeft( true, true );
    sg::ImageView glImgRight( true, true );
    sg::ImageView glDepth( true, false );

    glImgLeft.SetBounds( 3.0 / 4.0, 1.0, 0.0, 1.0 / 3.0, (double)g_nImgWidth / g_nImgHeight );
    glImgRight.SetBounds( 3.0 / 4.0, 1.0, 1.0 / 3.0, 2.0 / 3.0, (double)g_nImgWidth / g_nImgHeight );
    glDepth.SetBounds( 3.0 / 4.0, 1.0, 2.0 / 3.0, 1.0, (double)g_nImgWidth / g_nImgHeight );

    // Add our views as children to the base container.
    glBaseView.AddDisplay( glView3D );
    glBaseView.AddDisplay( glImgLeft );
    glBaseView.AddDisplay( glImgRight );
    glBaseView.AddDisplay( glDepth );


    // register key callbacks
    // pango::RegisterKeyPressCallback( 't', boost::bind( _StartLclzr ) );

    // launch tracker thread
    boost::thread TrackerThread( _TrackerLoop, pCam, &glImgLeft, &glImgRight, &glDepth );

	// //////////////////////////////////////////////////////////////////////////////
	// Screen Rendering
	//
    while( !pango::ShouldQuit() ) {

        // clear whole screen
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

        // swap frames and process events
        pango::FinishGlutFrame();

        // 60 Hz refresh rate
        usleep( 1E6 / 60 );
    }

    return 0;
}

// TODO: make a different thread for the minimization similar to SlamEngine
// TODO: load ground truth values