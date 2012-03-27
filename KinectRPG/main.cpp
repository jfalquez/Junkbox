#include <vector>

#include <pcl/pcl_base.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <cv.h>
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>

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
//	pcl::RangeImagePlanar depthImg;
//	pcl::visualization::RangeImageVisualizer DepthImgViewer ("Depth Image");
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud ( new pcl::PointCloud<pcl::PointXYZ> );
	pcl::visualization::CloudViewer PtCloudViewer("Cloud Viewer");


	while(1) {
		if( !Cam.Capture(vImages) ) {
			std::cout << "Error getting images." << std::endl;
		}

		cv::imshow("RGB Image", vImages[0]);

		const unsigned short* depth_map = (unsigned short*)vImages[1].data;
		int width = vImages[1].cols;
		int height = vImages[1].rows;
		float center_x = width/2;
		float center_y = height/2;
		float focal_length_x = Cam.GetProperty<float>("DepthFocalLength", 580);
		float focal_length_y = focal_length_x;

//		depthImg.setDepthImage (depth_map, width, height, center_x,	center_y,focal_length_x, focal_length_y);

//		DepthImgViewer.showRangeImage(depthImg);

		pCloud->clear();
		for( int ii = 0; ii <  height; ii ++ ) {
			for( int jj = 0; jj <  width; jj ++ ) {
				pcl::PointXYZ Point;
				float depth = vImages[1].at<unsigned short>(ii,jj);
				Point.y = -depth * ((center_x - ii) / focal_length_x);
				Point.x = depth * ((center_y - jj) / focal_length_x);
				Point.z = depth;
				pCloud->push_back(Point);
			}
		}

//		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> nEst;
//		nEst.setInputCloud ( pCloud );

		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree (new pcl::search::KdTree<pcl::PointXYZ > ());
//		nEst.setSearchMethod (kdTree);

		// Output datasets
//		pcl::PointCloud<pcl::Normal>::Ptr pCloudNormals (new pcl::PointCloud<pcl::Normal>);

		// Use all neighbors in a sphere of radius 3cm
//		nEst.setRadiusSearch (0.03);

		// Compute the features
//		nEst.compute (*pCloudNormals);

		PtCloudViewer.showCloud( pCloud );

		double leafSize = 0.01;
		double isoLevel = 0.5;

//		pcl::PolygonMesh mesh;
//		pcl::MarchingCubesGreedy<pcl::PointNormal> mc;
//		mc.setInputCloud(cloud_with_normals);
//		mc.setSearchMethod(tree2);
//		mc.setLeafSize(leafSize);
//		mc.setIsoLevel(isoLevel);
//		mc.reconstruct (mesh);


		char c;
		c = cv::waitKey(2);
		if (c == 27) break;

	}
	return 0;
}
