#include <vector>

#include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/range_image/range_image_planar.h>
#include <pcl/visualization/range_image_visualizer.h>

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
	std::vector< cv::Mat > vImages;

	// create GUI windows
	cv::namedWindow( "RGB Image", CV_WINDOW_AUTOSIZE );
	cv::namedWindow( "Depth Image", CV_WINDOW_AUTOSIZE );
	pcl::RangeImagePlanar depthImg;
	pcl::visualization::RangeImageVisualizer DepthImgViewer ("Depth Image");

	while(1) {
		if( !Cam.Capture(vImages) ) {
			std::cout << "Error getting images." << std::endl;
		}

		cv::imshow("RGB Image", vImages[0]);

		cv::imshow("Depth Image", vImages[1]);

		const unsigned short* depth_map = (unsigned short*)vImages[1].data;
		int width = vImages[1].cols;
		int height = vImages[1].rows;
		float center_x = width/2;
		float center_y = height/2;
		float focal_length_x = Cam.GetProperty<float>("DepthFocalLength", 580);
		float focal_length_y = focal_length_x;

		depthImg.setDepthImage (depth_map, width, height, center_x,
								center_y,focal_length_x, focal_length_y);

		DepthImgViewer.showRangeImage(depthImg);

		char c;
		c = cv::waitKey(2);
		if (c == 27) break;
	}
	return 0;
}
