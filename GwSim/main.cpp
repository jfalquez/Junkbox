/********************************************************
 * Basic Code
 *
 ********************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <RPG/Devices/Camera/CameraDevice.h>
#include <RPG/Robots/Robot.h>


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// da main
int main( int argc, char** argv )
{
//	rpg::Robot Agnos;
//	Agnos.Init("../../RPG/dev/jmf/GwSim/rpg_config.xml");


    // init driver
	CameraDevice Cam;
	Cam.InitDriver("NodeCam");
	std::vector< cv::Mat >   Images;       // Image to show on screen

    while(1) {
        if( Cam.Capture(Images) ) {
			cv::imshow( "SimL", Images[0] );
			cv::imshow( "SimR", Images[1] );
		}
		char c;
		c = cv::waitKey(2);
		if (c == 27) break;
    }
    return 0;
}