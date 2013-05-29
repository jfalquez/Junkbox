#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>

#include <HAL/Camera/CameraDevice.h>
#include <HAL/Utils/GetPot>

#include "Thumbnails.h"

int main(int argc, char** argv)
{
    GetPot clArgs(argc,argv);

    hal::Camera Cam( clArgs.follow("", "-cam" ) );

    pb::ImageArray Imgs;

    std::vector< cv::Mat > vBuff;

    Thumbnails Thmbs;

    for(unsigned int nFrame = 0; ; ++nFrame ) {

        Cam.Capture( Imgs );

        cv::Mat Img( Imgs[0] );

        for( int ii = 0; ii < 5; ++ii ) {
            cv::Mat Tmp;
            cv::resize( Img, Tmp, cv::Size(0,0), 0.5, 0.5 );
            Img = Tmp;
        }

        vBuff.push_back( Img );

        std::vector< unsigned int >    vMatches;
        Thmbs.FindBestMatch( vBuff[vBuff.size()-1].data, nullptr, vMatches );

        std::cout << "Matches for " << nFrame << ":" << std::endl;
        for( int ii = 0; ii < vMatches.size(); ++ii ) {
            std::cout << vMatches[ii] << ", ";
        }
        std::cout << std::endl;

        Thmbs.PushThumbnails( nFrame, Img.rows * Img.cols, vBuff[vBuff.size()-1].data );


        ///----------------------------------------------------


        // clear whole screen
//        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

//        pangolin::FinishGlutFrame();

        std::cout << "."; fflush(stdout);
        sleep(1);
    }

    return 0;
}
