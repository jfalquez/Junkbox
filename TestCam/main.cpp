#include <RPG/Utils/TicToc.h>
#include <RPG/Devices/Camera/CameraDevice.h>


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// da main
int main( int argc, char** argv )
{

    CameraDevice Cam;

    // set properties.. if any
    //Cam.SetProperty("NumImages", 2);
    //Cam.SetProperty("ImageWidth", 640);
    //Cam.SetProperty("ImageHeight", 480);

    Cam.SetProperty( "NumNodes", 1 );
//    Cam.SetProperty( "Node-0", "localhost:5556" );
    Cam.SetProperty( "Node-0", "192.168.1.4:5556" );

    // init driver
    if( !Cam.InitDriver( "NodeCam" ) ) {
        std::cout << "Invalid input device." << std::endl;
        return -1;
    }

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
        std::string NodeName = vImages[0].Map.GetProperty("NodeName");
        double Timestamp = vImages[0].Map.GetProperty<double>("CameraTime");
//        printf("%s %f\n", NodeName.c_str(), Timestamp );
        cv::imshow( "Image 2", vImages[1].Image );

        char c;
//        c = cv::waitKey(2);
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
