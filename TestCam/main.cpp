
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
    Cam.SetProperty( "Node-0", "localhost:5556" );

    // init driver
    if( !Cam.InitDriver( "NodeCam" ) ) {
        std::cout << "Invalid input device." << std::endl;
        return -1;
    }

    std::cout << "Success." << std::endl;

    // container for images
    std::vector< rpg::ImageWrapper > vImages;

    // create GUI windows
    cv::namedWindow( "Image 1", CV_WINDOW_AUTOSIZE );
    //cv::namedWindow( "Image 2", CV_WINDOW_AUTOSIZE );


    while(1) {
        if( !Cam.Capture(vImages) ) {
            std::cout << "Error getting images." << std::endl;
        }

        std::cout << "Captured..." << std::endl;

//        cv::imshow( "Image 1", vImages[0].Image );
        //cv::imshow("Image 2", vImages[1]);

        char c;
        c = cv::waitKey(2);
        if (c == 27) break;

    }
    return 0;
}
