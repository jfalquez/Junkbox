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


std::vector< cv::Mat >      Images;       // Image to show on screen
CameraDevice                Cam;            // The camera handle


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// da main
int main( int argc, char** argv )
{
    // init driver
    if( !Cam.InitDriver( "GwSim" ) ) {
    	std::cout << "Invalid input device." << std::endl;
    	return -1;
    }
    
    while(1) {
        Cam.Capture(Images);
        cv::imshow( "GwSim", Images[0] );
    }
    return 0;
}