#include <RPG/Utils/InitCam.h>
#include <RPG/Utils/TicToc.h>
#include <RPG/Devices/Camera/CameraDevice.h>


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// da main
int main( int argc, char** argv )
{

    CameraDevice Cam;

    rpg::InitCam( Cam, argc, argv );

    // container for images
    std::vector< rpg::ImageWrapper > vImages;

    // create GUI windows
    cv::namedWindow( "Image 1", CV_WINDOW_AUTOSIZE );
    cv::namedWindow( "Image 2", CV_WINDOW_AUTOSIZE );


    // variable to calculate frame rate
    double t = Tic();
    unsigned int nFrames = 0;

    while(1) {
        if( !Cam.Capture(vImages) ) {
            std::cout << "Error getting images." << std::endl;
        }

        cv::imshow( "Image 1", vImages[0].Image );
        cv::imshow( "Image 2", vImages[1].Image );

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
