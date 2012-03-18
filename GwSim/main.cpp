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


std::vector< cv::Mat >      Images;       // Image to show on screen

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// da main
int main( int argc, char** argv )
{
	rpg::Robot Agnos;
	Agnos.Init("../../RPG/dev/jmf/GwSim/rpg_config.xml");
	// read URDF file

    // init driver

    while(1) {
//        Cam.Capture(Images);
//        cv::imshow( "GwSim", Images[0] );
    }
    return 0;
}