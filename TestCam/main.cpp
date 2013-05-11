#include <unistd.h>

#include <opencv.hpp>

#include <HAL/Utils/TicToc.h>
#include <HAL/Camera/CameraDevice.h>


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// da main
int main( int argc, char** argv )
{

    hal::CameraDevice Cam;
    std::cout << "Camera ok..." << std::endl;

    if( Cam.InitDriver( argc, argv ) == false ) {
        std::cerr << "Error initializing driver." << std::endl;
        exit(1);
    }
    std::cout << "Init ok..." << std::endl;

    // create GUI windows
    cv::namedWindow( "Image 1", CV_WINDOW_AUTOSIZE );
    cv::namedWindow( "Image 2", CV_WINDOW_AUTOSIZE );


    // variable to calculate frame rate
    double t = Tic();
    unsigned int nFrames = 0;

    pb::ImageArray vImages;

    while(1) {
        if( !Cam.Capture(vImages) ) {
            std::cout << "Error getting images." << std::endl;
        }

        cv::imshow( "Image 1", vImages(0).cvMat() );
        cv::Mat Depth;
        vImages(1).cvMat().convertTo( Depth, CV_32FC1 );
        Depth = Depth / 1000;
        Depth = Depth / 10.0;
        cv::imshow( "Image 2", Depth );

        char c;
        c = cv::waitKey(1);
        if (c == 27) break;

        nFrames++;
        double dTimeLapse = Toc(t);
        if( dTimeLapse > 1.0 ){
            printf( "Framerate: %.2f\r", nFrames/dTimeLapse );
            fflush(stdout);
            nFrames  = 0;
            t = Tic();
        }
    }
    return 0;
}
