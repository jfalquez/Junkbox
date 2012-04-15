/********************************************************
 * Basic Code
 *
 ********************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <sys/time.h>

#include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <RPG/Devices/Camera/CameraDevice.h>
//#include <RPG/Robots/Robot.h>


// Aux Time Functions
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////
inline double Tic()
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    return tv.tv_sec + 1e-6 * (tv.tv_usec);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////
inline double RealTime()
{
    return Tic();
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////
inline double Toc( double  dTic )
{
    return Tic() - dTic;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline double TocMS( double  dTic )
{
    return ( Tic() - dTic )*1000.;
}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// da main
int main( int argc, char** argv )
{
//	rpg::Robot Agnos;
//	Agnos.Init("../../RPG/dev/jmf/GwSim/rpg_config.xml");


    // init driver
	CameraDevice Cam;
	Cam.SetProperty("Host", "localhost:6666");
	Cam.InitDriver("NodeCam");
	std::vector< cv::Mat >   Images;       // Image to show on screen


	long unsigned int count = 0;
	double t = Tic();

    while(1) {
        if( Cam.Capture(Images) ) {
			cv::imshow( "SimL", Images[0] );
//			cv::imshow( "SimR", Images[1] );
		}


		count ++;
		if(count > 5000) {
			break;
		}


		char c;
		c = cv::waitKey(2);
		if (c == 27) break;
    }

	double time = Toc(t);
	printf("Framerate %.2f\n",count/time);


    return 0;
}