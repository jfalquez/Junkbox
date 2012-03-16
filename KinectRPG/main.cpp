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
Cam.SetProperty( "ConfigFile", "ConfigFile.xml");    

// init driver
    if( !Cam.InitDriver( "Kinect" ) ) {
    	std::cout << "Invalid input device." << std::endl;
    	return -1;
    }

std::vector< cv::Mat > vImages;

while(1) {
if( !Cam.Capture(vImages) ) {
	std::cout << "Error getting images." << std::endl;
}

cv::namedWindow( "RGB Image", CV_WINDOW_AUTOSIZE );
cv::namedWindow( "Depth Image", CV_WINDOW_AUTOSIZE );
cv::imshow("RGB Image", vImages[0]);
cv::imshow("Depth Image", vImages[1]);
char c;
c = cv::waitKey(2);
if (c == 27) break;
}
    return 0;
}
