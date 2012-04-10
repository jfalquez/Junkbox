#include <vector>

#include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <RPG/Devices/Camera/CameraDevice.h>


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// da main
int main( int argc, char** argv )
{

	CameraDevice Cam;

	// set properties
	Cam.SetProperty("NumImages", 2);
	Cam.SetProperty("ImageWidth", 640);
	Cam.SetProperty("ImageHeight", 480);

	// init driver
	if( !Cam.InitDriver( "HDMI" ) ) {
		std::cout << "Invalid input device." << std::endl;
		return -1;
	}

	exit(0);

	// container for images
	std::vector< cv::Mat > vImages;

	// create GUI windows
	cv::namedWindow( "Image 1", CV_WINDOW_AUTOSIZE );
	cv::namedWindow( "Image 2", CV_WINDOW_AUTOSIZE );


	while(1) {
		if( !Cam.Capture(vImages) ) {
			std::cout << "Error getting images." << std::endl;
		}

		cv::imshow("Image 1", vImages[0]);
		cv::imshow("Image 2", vImages[1]);

		char c;
		c = cv::waitKey(2);
		if (c == 27) break;

	}
	return 0;
}
