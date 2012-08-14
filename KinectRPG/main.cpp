#include <vector>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <RPG/Devices/Camera/CameraDevice.h>


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// da main
int main( int argc, char** argv )
{

	CameraDevice Cam;

	// init driver
	if( !Cam.InitDriver( "Kinect" ) ) {
		std::cout << "Invalid input device." << std::endl;
		return -1;
	}

	// container for Kinect images
   	std::vector<rpg::ImageWrapper> vImages;

	// create GUI windows
	cv::namedWindow( "RGB Image", CV_WINDOW_AUTOSIZE );

	while(1) {
		if( !Cam.Capture(vImages) ) {
			std::cout << "Error getting images." << std::endl;
		}

		cv::imshow("RGB Image", vImages[0].Image);
		cv::imshow("Depth Image", vImages[1].Image);

		const unsigned short* depth_map = (unsigned short*)vImages[1].Image.data;
		int width = vImages[1].Image.cols;
		int height = vImages[1].Image.rows;
		float center_x = width/2;
		float center_y = height/2;
		float focal_length_x = Cam.GetProperty<float>("DepthFocalLength", 580);
		float focal_length_y = focal_length_x;


        // clear Pt cloud
		for( int ii = 0; ii <  height; ii ++ ) {
			for( int jj = 0; jj <  width; jj ++ ) {
				float depth = vImages[1].Image.at<unsigned short>(ii,jj);
				float y = -depth * ((center_x - ii) / focal_length_x);
				float x = depth * ((center_y - jj) / focal_length_x);
				float z = depth;
//				pCloud->push_back(Point);
			}
		}


		char c;
		c = cv::waitKey(2);
		if (c == 27) break;

	}
	return 0;
}
