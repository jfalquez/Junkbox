#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <RPG/Devices/Camera/CameraDevice.h>
#include <RPG/Robots/Robot.h>

#include "Node.h"



void PackImages(
		std::vector< cv::Mat >& vImages,	//< Input:
		zmq::message_t& Msg					//< Output:
		)
	{
	int NumImages = vImages.size();

	// calculate total message size to allocate
	int MsgSize = 4;
	for(int ii = 0; ii < NumImages; ii++ ) {
		int ImgSize = vImages[ii].rows * vImages[ii].cols * vImages[ii].elemSize();
		MsgSize += 12 + ImgSize;	// 12 bytes = (4)Width, (4)Height and (4)ImgType
	}
	Msg.rebuild( MsgSize );

	// get message pointer
	char* MsgPtr = (char*)Msg.data();

	// push number of images in message
	memcpy( MsgPtr, &NumImages, sizeof(NumImages) );
	MsgPtr += sizeof(NumImages);

	// push images in width, height, type, data format
	for(int ii = 0; ii < NumImages; ii++ ) {
		int ImgWidth = vImages[ii].cols;
		int ImgHeight = vImages[ii].rows;
		int ImgType = vImages[ii].type();
		int ImgSize = ImgWidth * ImgHeight * vImages[ii].elemSize();

		memcpy( MsgPtr, &ImgWidth, sizeof(ImgWidth) );
		MsgPtr += sizeof(ImgWidth);
		memcpy( MsgPtr, &ImgHeight, sizeof(ImgHeight) );
		MsgPtr += sizeof(ImgHeight);
		memcpy( MsgPtr, &ImgType, sizeof(ImgType) );
		MsgPtr += sizeof(ImgType);

		memcpy( MsgPtr, vImages[ii].data, ImgSize );
		MsgPtr += ImgSize;
	}

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// da main
int main( int argc, char** argv )
{
//	rpg::Robot Agnos;
//	Agnos.Init("../../RPG/dev/jmf/GwSim/rpg_config.xml");


    // init driver
	CameraDevice Cam;
	Cam.SetProperty("DataSourceDir", "/home/jmf/Workspace/datasets/CityBlock-Noisy" );
	Cam.SetProperty("Channel-0",     "left_rect.*" );
	Cam.SetProperty("Channel-1",     "right_rect.*" );
	Cam.SetProperty("NumChannels",   2 );
	Cam.InitDriver("FileReader");

	std::vector< cv::Mat >   Images;       // Image to show on screen

	rpg::Node Sender(7777);
	Sender.Publish( "CamFeed", 5556 );
	zmq::message_t Msg;

    while(1) {
        Cam.Capture(Images);
		PackImages( Images, Msg );
		Sender.Write( "CamFeed", Msg );
		std::cout << "." << std::flush;
		sleep(1);
    }
    return 0;
}