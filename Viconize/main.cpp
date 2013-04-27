#include <RPG/Utils/InitCam.h>
#include <RPG/Devices/Camera/CameraDevice.h>


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// da main
int main( int argc, char** argv )
{

    CameraDevice Cam;

    rpg::InitCam( Cam, argc, argv );

    // container for images
    std::vector< rpg::ImageWrapper > vImages;

    while(1) {
        if( !Cam.Capture(vImages) ) {
            std::cout << "Error getting images." << std::endl;
        }

    }
    return 0;
}
