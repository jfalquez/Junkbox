#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>

#include <opencv2/highgui/highgui.hpp>  // for imread()

#include <RPG/Devices/Camera/CameraDevice.h>

#include <boost/bind.hpp>

//#include <stdlib.h>
//#include <cstring>
//#include <string>
//#include <sys/time.h>
//#include <deque>
//#include <fstream>
//#include <iomanip>
//#include <iostream>
//#include <thread>
//#include <tuple>

//#include <fcntl.h>
//#include <sys/mman.h>
//#include <sys/ioctl.h>
#include <RPG/Utils/GetPot>
#include <RPG/Utils/TicToc.h>
#include "DiskLogger.h"

using namespace std;
namespace sg =SceneGraph;
namespace pango =pangolin;


// //////////////////////////////////////////////////////////////////////////////

unsigned int             g_nImgWidth = 640;
unsigned int             g_nImgHeight = 480;
unsigned int             g_nImgSize = 2;

CameraDevice             g_Cam;
DiskLogger               g_Logger( &g_Cam,"/Users/jmf/Code/Builds/StereoLogger/images", "l" ,"r",".ppm" );

const unsigned int DiskLogger::g_nPaddingImageNumber = 6;
const unsigned int DiskLogger::g_nPaddingCameraNumber = 2;

// //////////////////////////////////////////////////////////////////////////////
void _Capture()
{
	g_Logger.Start(0);
}


// //////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    GetPot cl( argc, argv );

	// for file reader
	string sDeviceDriver     = cl.follow( "FileReader", 1, "-idev" );
	string sLeftCameraModel  = cl.follow( "lcmod.xml", 1, "-lcmod" );
	string sRightCameraModel = cl.follow( "rcmod.xml", 1, "-rcmod" );
	string sLeftFileRegex    = cl.follow( "left.*pgm", 1, "-lfile" );
	string sRightFileRegex   = cl.follow( "right.*pgm", 1, "-rfile" );
	string sSourceDir        = cl.follow( ".", 1, "-sdir"  );


	// ideally pass this by command line
	// for now, hardcode it
	g_Cam.SetProperty<int>("NumImages", 2);
    g_Cam.SetProperty<int>("ImageWidth", g_nImgWidth);
    g_Cam.SetProperty<int>("ImageHeight", g_nImgHeight);

    // init driver
    if( !g_Cam.InitDriver( "Webcam" ) ) {
            cout << "Invalid input device." << endl;
            return -1;
    }

    // Create OpenGL window in single line thanks to GLUT
    pango::CreateGlutWindowAndBind( "Disk Logger", 2*g_nImgWidth, g_nImgHeight );
    sg::GLSceneGraph::ApplyPreferredGlSettings();

    // Scenegraph to hold GLObjects and relative transformations
    sg::GLSceneGraph glGraph;

    // Define Camera Render Object (for view / scene browsing)
    pango::OpenGlRenderState glState( pango::ProjectionMatrix( 640, 480, 420, 420, 320, 240, 0.1, 1000 ),
                                      pango::ModelViewLookAt( -6, 0, -30, 1, 0, 0, pango::AxisNegZ ) );

    // Pangolin abstracts the OpenGL viewport as a View.
    // Here we get a reference to the default 'base' view.
    pango::View& glBaseView = pango::DisplayBase();

    // display images
    sg::ImageView glLeftImg( false, true );

    glLeftImg.SetBounds( 0.0, 1.0, 0.0, 0.5, (double)g_nImgWidth / g_nImgHeight );

    sg::ImageView glRightImg( false, true );

    glRightImg.SetBounds( 0.0, 1.0, 0.5, 1.0, (double)g_nImgWidth / g_nImgHeight );

    // Add our views as children to the base container.
    glBaseView.AddDisplay( glLeftImg );
    glBaseView.AddDisplay( glRightImg );

    // register key callbacks
    pango::RegisterKeyPressCallback( 'c', boost::bind( _Capture ) );

	// vector of images
    std::vector<rpg::ImageWrapper> vImages;

	// variable to calculate frame rate
    double t = Tic();
    unsigned int nFrames = 0;

    // Default hooks for exiting (Esc) and fullscreen (tab).
    while( !pango::ShouldQuit() ) {

		// capture images
        g_Cam.Capture(vImages);
		g_nImgHeight = vImages[0].Image.rows;
		g_nImgWidth = vImages[0].Image.cols;

		// clear whole screen
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

        // show left image
		glLeftImg.SetImage( vImages[0].Image.data, g_nImgWidth, g_nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_UNSIGNED_BYTE );

        // show right image
		glRightImg.SetImage( vImages[1].Image.data, g_nImgWidth, g_nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_UNSIGNED_BYTE );

        // swap frames and Process Events
        pango::FinishGlutFrame();

        nFrames++;
        double dTimeLapse = Toc(t);
        if( dTimeLapse > 1.0 ){
            printf( "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\bFramerate %.2f", nFrames/dTimeLapse );
            fflush(stdout);
            nFrames  = 0;
            t = Tic();
        }

        // pause for 1/60th of a second.
        usleep( 1E6 / 60 );
    }

    return 0;
}

