/********************************************************
 *
 *
 ********************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <vector>

#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/gl.h>
#include <FLConsole/FLConsole.h>
#include "GLHelpers.h"

#include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <RPG/Utils/GetPot>
#include <RPG/Devices/Camera/CameraDevice.h>

#include <Mvlpp/Utils.h>

using namespace cv;


const char USAGE[] =
"Usage:     slam -idev <input> <options>\n"
"\n"
"where input device can be: FileReader Bumblebee2 etc\n"
"\n"
"Input Specific Options:\n"
"   FileReader:      -lfile <regular expression for left image channel>\n"
"                    -rfile <regular expression for right image channel>\n"
"\n"
"General Options:    -lcmod <left camera model xml file>\n"
"                    -rcmod <right camera model xml file>\n"
"\n"
"Example:\n"
"slam -idev FileReader -lcmod lcmod.xml -rcmod rcmod.xm -lfile \"left*pgm\"  -rfile \"right*.pgm\"\n\n";


/********************************************************
 * Function Stubs										*
 ********************************************************/
void tracker_Main( void );

int tracker_Export( const vector<Mat>vImages,
					vector<KeyPoint>& keypointsLeft,
					vector<KeyPoint>& keypointsRight,
					vector<DMatch>& matches );

void tracker_DescriptorExtractor( const Mat& image,
								  vector<KeyPoint>& keypoints,
								  Mat& descriptors );

void tracker_Matcher(const vector<KeyPoint>& queryK,
					 const Mat& queryD,
					 const vector<KeyPoint>& trainK,
					 const Mat& trainD,
					 vector<DMatch>& matches );

int loadTexture_Mat( Mat image, GLuint *text );



/********************************************************
 * Classes, Global Variables & Constants				*
 ********************************************************/
int&  window =
		CVarUtils::CreateCVar( "window", 2, "Area around feature pixel to compute descriptor. It follows: ( 2 * window ) + 1" );
int&  sArea =
		CVarUtils::CreateCVar( "sArea", 10, "Search area size for matching features." );
int&  tFactor =
		CVarUtils::CreateCVar( "tFactor", 4, "Threshold factor for matching features. New threshold "
									   	   	 "would be tFactor * min_distance." );
bool& DRAW_ALL_LEFT =
		CVarUtils::CreateCVar( "drawAll.left", false, "Draw all keypoints (left image) regarding if "
													  "they match or not (toggle).");
bool& DRAW_ALL_RIGHT =
		CVarUtils::CreateCVar( "drawAll.right", false, "Draw all keypoints (right image) regarding if "
													   "they match or not (toggle).");
bool& f_PAUSED =
		CVarUtils::CreateCVar( "pause", false, "Pause data capture (toggle)." );

bool f_EXPORT = false;

#define FPS (1.0 / 10.0)					// Refresh rate, how fast it reads images from disk

Mat				showImg;					// Image to show on screen
CameraDevice 	cam;						// The camera handle

unsigned int exportID = 0;					// ID to keep track of data exporting (ie. image_000, etc).

// FLTK OpenGL window
class GLWindow : public Fl_Gl_Window
{

	static void Timer(void *userdata)
	{
		GLWindow *pWin = (GLWindow*)userdata;
		if ( !f_PAUSED ) {
			tracker_Main();
		}
		pWin->redraw();
		Fl::repeat_timeout( FPS, Timer, userdata );
	}

public:

	GLWindow( int x, int y, int w, int h, const char *l=0 ) : Fl_Gl_Window( x, y, w, h, l )
	{
		Fl::add_timeout( FPS, Timer, (void*)this );
	}

	void draw()
	{
        GLuint texture;

        // handle reshaped viewport
        if ( !valid() ) {
        	ReshapeViewport( w(), h() );
        }

        // Initialization
        static bool bInitialized = false;
        if ( !bInitialized || !context_valid() ) {
            bInitialized = true;
            glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
            glClearDepth(1.0f);
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LEQUAL);
            glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
        }

    	loadTexture_Mat( showImg, &texture );

        PushOrtho( w(), h() );
        glDisable( GL_TEXTURE_2D );
        glEnable( GL_TEXTURE_RECTANGLE_ARB );
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL );
        glBindTexture( GL_TEXTURE_RECTANGLE_ARB, texture );
        OrthoQuad( showImg.cols, showImg.rows, 0, 0, h(), w() );
        glDisable( GL_TEXTURE_RECTANGLE_ARB );
        PopOrtho();

        m_Console.draw();
	}

	int handle( int event )
	{
		if ( context() ) {
			if( m_Console.IsOpen() ) {
				return m_Console.handle( event );
			}
			else {
				switch ( event ) {
				case FL_FOCUS:
				case FL_UNFOCUS:
				case FL_KEYBOARD:
				switch( Fl::event_key() ) {
					case '`':
						m_Console.OpenConsole();
						return 1;
					case 'q':
						exit(1);
					case 'c':
						f_EXPORT = true;
						return 1;
					case ' ':
						if( f_PAUSED )
							f_PAUSED = false;
						else
							f_PAUSED = true;
						return 1;
					}
				break;
				}
			}
		}
		return false;
	}


private:

	FLConsoleInstance m_Console;

};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// da main
int main( int argc, char** argv )
{
    if( argc < 3 ) {
        std::cout << USAGE;
        return -1;
    }

    GetPot cl(argc,argv);

    // get input device
    string sInputDevice  = cl.follow( "", 1, "-idev" );


    if( sInputDevice == "Bumblebee2" ) {
        string sLeftCameraModel  = cl.follow( "", 1, "-lcmod" );
        string sRightCameraModel = cl.follow( "", 1, "-rcmod" );
        if( sLeftCameraModel.empty() && sRightCameraModel.empty() ) {
            std::cout << "One or more camera model files are missing!\n" << std::endl;
            std::cout << USAGE;
            return -1;
        }
        cam.SetProperty("CamModel-L", sLeftCameraModel );
        cam.SetProperty("CamModel-R", sRightCameraModel );
    }


    if( sInputDevice == "FileReader" ) {
        string sLeftImageFile    = cl.follow( "", 1, "-lfile" );
        string sRightImageFile   = cl.follow( "", 1, "-rfile" );
        if( sLeftImageFile.empty() && sRightImageFile.empty() ) {
            std::cout << "One or more file names is missing!\n" << std::endl;
            std::cout << USAGE;
            return -1;
        }
        cam.SetProperty("Channel-0", sLeftImageFile );
        cam.SetProperty("Channel-1", sRightImageFile );
        cam.SetProperty("NumChannels", 2 );
    }

    // init driver
    if( !cam.InitDriver( sInputDevice ) ) {
    	std::cout << "Invalid input device." << std::endl;
    	return -1;
    }

	// run once to have an initial image
	tracker_Main();

	GLWindow* pWin = new GLWindow( 0, 0, 1024, 384, "SLAM" );
	pWin->end();
    pWin->resizable( pWin );
    pWin->show();
    return( Fl::run() );
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// main feature matcher code
void tracker_Main( void )
{
    // get images
    std::vector<Mat> vImages;
    if( !cam.Capture( vImages ) )
    	exit(1);


    // get keypoints and compute descriptors
	FastFeatureDetector detector( 20 );						// FAST feature extractor
	std::vector<KeyPoint> keypointsLeft, keypointsRight;	// Feature vector
    detector.detect( vImages[0], keypointsLeft );
    detector.detect( vImages[1], keypointsRight );

    // compute descriptors
	Mat descriptorsLeft, descriptorsRight;					// Descriptor matrix
    tracker_DescriptorExtractor( vImages[0], keypointsLeft, descriptorsLeft );
    tracker_DescriptorExtractor( vImages[1], keypointsRight, descriptorsRight );

    // match keypoints based on descriptors
	std::vector<DMatch> matches;
    // match left to right
	tracker_Matcher( keypointsLeft, descriptorsLeft, keypointsRight, descriptorsRight, matches );

    // find distances between matches
    double max_dist = 0; double min_dist = 1000;
    for( int i = 0; i < descriptorsLeft.rows; i++ ) {
		double dist = matches[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
    }

    // select only "good" matches (i.e. distance less threshold)
    // if there is an exact match, min_dist will be 0
    // change to 1 to accept a slightly higher threshold
	std::vector<DMatch> good_matches;						// Good matches
    if( min_dist == 0 )
    	min_dist = 1;
    for( int i = 0; i < descriptorsLeft.rows; i++ ) {
		if( matches[i].distance <= tFactor * min_dist )
		    good_matches.push_back( matches[i] );
    }

	// export data (save to disk)
	if( f_EXPORT ) {
		if( tracker_Export( vImages, keypointsLeft, keypointsRight, good_matches ) ) {
			std::cout << "Data " << exportID - 1 << " exported succesfully." << std::endl;
		}
		f_EXPORT = false;
	}


    if ( DRAW_ALL_LEFT )
        drawKeypoints( vImages[0], keypointsLeft, vImages[0], Scalar::all(255), DrawMatchesFlags::DEFAULT );

    if ( DRAW_ALL_RIGHT )
    	drawKeypoints( vImages[1], keypointsRight, vImages[1], Scalar::all(255), DrawMatchesFlags::DEFAULT );

    // draw only "good" matches
    drawMatches( vImages[0], keypointsLeft, vImages[1], keypointsRight,
            good_matches, showImg, Scalar::all(-1), Scalar::all(-1),
            vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    // house cleaning
	matches.clear();
	good_matches.clear();
	keypointsLeft.clear();
	keypointsRight.clear();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// save data to disk
int tracker_Export( const vector<Mat>vImages,
					 vector<KeyPoint>& keypointsLeft,
					 vector<KeyPoint>& keypointsRight,
					 vector<DMatch>& matches )
{
    char filename[100];

	std::cout << "Keypoints in " << keypointsLeft.size() << std::endl;
    // surf descriptors
	SurfDescriptorExtractor extractor;
	Mat SurfDescriptors;
    extractor.compute( vImages[0], keypointsLeft, SurfDescriptors );
	std::cout << "Keypoints out " << keypointsLeft.size() << std::endl;

    // export images
    sprintf(filename,"img_left-%03d.png",exportID);
    if( !imwrite( filename, vImages[0] ) ) {
    	std::cout << "export: Error saving left image " << filename << std::endl;
    	return false;
    }
    sprintf(filename,"img_right-%03d.png",exportID);
    if( !imwrite( filename, vImages[1] ) ) {
    	std::cout << "export: Error saving right image " << filename << std::endl;
    	return false;
    }

    // export coordinates and descriptors
    std::ofstream file;
    sprintf(filename,"features-%03d.txt",exportID);
    file.open (filename);
	for( vector<DMatch>::iterator i = matches.begin(); i != matches.end(); ++i ) {
	    file << keypointsLeft[i->queryIdx].pt.x << " ";
	    file << keypointsLeft[i->queryIdx].pt.y << " ";
	    file << keypointsLeft[i->queryIdx].pt.x - keypointsRight[i->trainIdx].pt.x << " ";
	    file << SurfDescriptors.row(i->queryIdx) << std::endl;
	}
	file.close();
	std::cout << "Good Matches: " << matches.size() << std::endl;
	exportID++;
	return true;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// compute descriptors for each feature/keypoint
void tracker_DescriptorExtractor( const Mat& image, vector<KeyPoint>& keypoints, Mat& descriptors )
{
	int oX, oY, numDescs, numElems, dArea;

	// dArea is the "real" search area for the descriptor. If window = 1, then dArea is 3 (ie. 3x3 area)
	dArea = (2 * window) + 1;
	numDescs = 0;
	descriptors.create( keypoints.size(), (dArea * dArea), CV_8UC1 );
	for( vector<KeyPoint>::iterator i = keypoints.begin(); i != keypoints.end(); ++i ) {
		// find matrix origin based on keypoint coordinate and dArea size
		oX = (int)i->pt.x - (dArea / 2);
		oY = (int)i->pt.y - (dArea / 2);

		// check to see if dArea is out of bounds
		if( oX < 0 || (oX + dArea) > image.cols ||
		    oY < 0 || (oY + dArea) > image.rows ) {
			// if so, delete... bad keypoint
			std::cout << "matcher: Bad keypoint.. deleted!" << std::endl;
			i = keypoints.erase(i);
		} else {
			// copy the neighborhood to the descriptors matrix
			numElems = 0;
			for( int j = 0; j < dArea ; j++ ) {
				for( int k = 0; k < dArea; k++ ) {
					descriptors.at<uchar>(numDescs, numElems) =
						image.at<uchar>(oY + j, oX + k);
					numElems++;
				}
			}
			numDescs++;
		}
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// feature/keypoint matching algorithm
void tracker_Matcher( const vector<KeyPoint>& queryK, const Mat& queryD, const vector<KeyPoint>& trainK,
					  const Mat& trainD, vector<DMatch>& matches )
{
	Mat res;
	int acc, minVal, minInd;
	
	// compare each descriptor in 'query' against 'data'
	for( int i = 0; i < queryD.rows; i++ ) {
		// initialize with some big value since we are looking for the closest match (minimum value)
		// if there are no features inside the search area (ie. this value will remain)
		// then this will be filtered out during the 'good match' procedure
		minInd = 0;
		minVal = 999999;
		for( int j = 0; j < trainD.rows; j++ ) {
			// now that the images are rectified you can use this 'y' comparison, before you needed
			// a threshold like the line below
			if( trainK[j].pt.x > (queryK[i].pt.x - sArea) && trainK[j].pt.x < (queryK[i].pt.x + sArea)
			  && trainK[j].pt.y == queryK[i].pt.y ) {
//			  && trainK[j].pt.y > (queryK[i].pt.y - sArea) && trainK[j].pt.y < (queryK[i].pt.y + sArea) ) {
				absdiff( queryD.row(i), trainD.row(j), res );
				// crappy method to sum all elements in a row
				acc = 0;
				for( int a = 0; a < res.cols; a++ ) acc += res.at<uchar>(a);
				if( acc < minVal ) {
					minVal = acc;
					minInd = j;
				}
			}
		}
		matches.push_back( DMatch(i, minInd, minVal) );
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// image Mat to OpenGl texture function/converter
int loadTexture_Mat(Mat image, GLuint *text)
{

    if( !image.data )
	    return -1;

    glGenTextures(1, text);

	glBindTexture( GL_TEXTURE_RECTANGLE_ARB, *text ); //bind the texture to it's array
//  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
//	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
//	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

 	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGB, image.cols, image.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, image.data);

    return 0;
}
