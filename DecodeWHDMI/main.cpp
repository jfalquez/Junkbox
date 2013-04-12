#include <RPG/Utils/InitCam.h>
#include <RPG/Utils/TicToc.h>
#include <RPG/Devices/Camera/CameraDevice.h>

#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>

using namespace std;


inline unsigned char DecodeByte( unsigned char* ptr )
{
    unsigned char byteData = 0;
    unsigned char bitData;

    bitData = (*(ptr) + *(ptr+1) + *(ptr+2) ) / 3;
    ptr = ptr+3;
    byteData = byteData | ( bitData > 128 ? 0x80 : 0 );

    bitData = (*(ptr) + *(ptr+1) + *(ptr+2) ) / 3;
    ptr = ptr+3;
    byteData = byteData | ( bitData > 128 ? 0x40 : 0 );

    bitData = (*(ptr) + *(ptr+1) + *(ptr+2) ) / 3;
    ptr = ptr+3;
    byteData = byteData | ( bitData > 128 ? 0x20 : 0 );

    bitData = (*(ptr) + *(ptr+1) + *(ptr+2) ) / 3;
    ptr = ptr+3;
    byteData = byteData | ( bitData > 128 ? 0x10 : 0 );

    bitData = (*(ptr) + *(ptr+1) + *(ptr+2) ) / 3;
    ptr = ptr+3;
    byteData = byteData | ( bitData > 128 ? 0x08 : 0 );

    bitData = (*(ptr) + *(ptr+1) + *(ptr+2) ) / 3;
    ptr = ptr+3;
    byteData = byteData | ( bitData > 128 ? 0x04 : 0 );

    bitData = (*(ptr) + *(ptr+1) + *(ptr+2) ) / 3;
    ptr = ptr+3;
    byteData = byteData | ( bitData > 128 ? 0x02 : 0 );

    bitData = (*(ptr) + *(ptr+1) + *(ptr+2) ) / 3;
    ptr = ptr+3;
    byteData = byteData | ( bitData > 128 ? 0x01 : 0 );

    return byteData;

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// da main
int main( int argc, char** argv )
{
    CameraDevice Cam;

    if( rpg::InitCam( Cam, argc, argv ) == false ) {
        std::cerr << "Error initializing camera!" << std::endl;
        exit(1);
    }

    // container for images
    std::vector< rpg::ImageWrapper > vImages;

    // capture an initial image to get image params
    Cam.Capture( vImages );

    // image info
    const unsigned int nNumImgs     = 2;
    const unsigned int nImgHeight   = 240;
    const unsigned int nImgWidth    = 320;

    if( nNumImgs == 0 ) {
        cerr << "No images found!" << endl;
        exit(0);
    }

    // Create OpenGL window in single line thanks to GLUT
    unsigned int nWinWidth = nNumImgs * nImgWidth;
    nWinWidth = nWinWidth > 1920? 1920 : nWinWidth;
    unsigned int nWinHeight = nImgHeight;
    nWinHeight = nWinHeight > 1080 ? 1080 : nWinHeight;
    pangolin::CreateGlutWindowAndBind( "DecoderWHDMI", nWinWidth, nWinHeight );
    SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();

     // pangolinlin abstracts the OpenGL viewport as a View.
    // Here we get a reference to the default 'base' view.
    pangolin::View& glBaseView = pangolin::DisplayBase();

    // display images
    SceneGraph::ImageView			 glLeftImg;
    SceneGraph::ImageView			 glRightImg;
    glLeftImg.SetBounds( 0.0, 1.0, 0.0, 0.5, (double)nImgWidth / nImgHeight );
    glRightImg.SetBounds( 0.0, 1.0, 0.5, 1.0, (double)nImgWidth / nImgHeight );

    // Add our views as children to the base container.
    glBaseView.AddDisplay( glLeftImg );
    glBaseView.AddDisplay( glRightImg );

    // variable to calculate frame rate
    double dTic = Tic();
    unsigned int nFrames = 0;

    // set up images
    cv::Mat ImgRGB( 240, 320, CV_8UC3 );
    cv::Mat ImgDepth( 240, 320, CV_16UC1 );

    // data points
    unsigned int ImageID = 0;
    unsigned int RGBImage = 1;
    unsigned int DepthImageLo = 80;
    unsigned int DepthImageHi = 200;

    // Default hooks for exiting (Esc) and fullscreen (tab).
    while( !pangolin::ShouldQuit() ) {

        // capture an images if not logging
        Cam.Capture( vImages );

        unsigned char* ptrID = vImages[0].Image.data + ( ImageID * 1280 * 3 );
        unsigned char* ptrRGB = vImages[0].Image.data + ( RGBImage * 1280 * 3);
        unsigned char* ptrDepthHi = vImages[0].Image.data + ( DepthImageHi * 1280 * 3);
        unsigned char* ptrDepthLo = vImages[0].Image.data + ( DepthImageLo * 1280 * 3);

        // get RGB image
//        memcpy( ImgRGB.data, ptrRGB, 320*240*3 );

        // set 0-1 image.. for measuring latency
        /*
        unsigned int VAL = (ImgRGB.data)[100] + (ImgRGB.data)[200] + (ImgRGB.data)[300] + (ImgRGB.data)[400] + (ImgRGB.data)[500] / 5;
        if( VAL < 128 ) {
            memcpy( ImgDepth.data, WHITE, 320*240*2 );
        } else {
            memcpy( ImgDepth.data, BLACK, 320*240*2 );
        }
        /* */

        // data comes as: high-nibble then low-nibble of first byte, then high-low of second byte
        // we'll average the BLUE, RED, GREEN to avoid bias towards a particular channel
        /*
        unsigned char byteData;
        unsigned short depthData;
        for( int ii = 0; ii < 240; ii++ ) {
            for( int jj = 0; jj < 320; jj++ ) {

                  depthData = DecodeByte( ptrDepthHi ) << 8;
                  ptrDepthHi += 24;

                  byteData = *ptrDepthLo++;
                  byteData += *ptrDepthLo++;
                  byteData += *ptrDepthLo++;
                  byteData = byteData / 3;
                  depthData += byteData;

                  ImgDepth.at<unsigned short>(ii,jj) = depthData;
            }
        }
        /**/


        // show left image
//        glLeftImg.SetImage( ImgRGB.data, nImgWidth, nImgHeight, GL_RGB8, GL_RGB, GL_UNSIGNED_BYTE );

        // show right image
//        glRightImg.SetImage( ImgDepth.data, nImgWidth, nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_UNSIGNED_SHORT );

        // swap frames and Process Events
        pangolin::FinishGlutFrame();

        nFrames++;
        double dTimeLapse = Toc( dTic );
//        if( dTimeLapse > 1.0 )
        {
            printf( "ID: %4d \t Framerate: %.2f\n", DecodeByte( ptrID ), nFrames/dTimeLapse );
            fflush(stdout);
            nFrames  = 0;
            dTic = Tic();
        }
    }

    return 0;
}

