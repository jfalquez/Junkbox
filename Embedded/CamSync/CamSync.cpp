#include <HAL/Utils/GetPot>
#include <HAL/Utils/TicToc.h>
#include <HAL/Camera/CameraDevice.h>

#include <fcntl.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/ioctl.h>


/***********************************************************************************
 * These methods assume the frame buffer is 24-bit in depth (RGB).
 ***********************************************************************************/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void SendLossyUC1(
        unsigned char*      pFb,
        unsigned int        nScreenWidth,
        unsigned char*      pImg,
        unsigned int        nImgHeight,
        unsigned int        nImgWidth,
        unsigned int        nTop,
        unsigned int        nLeft = 0
        )
{
    for( unsigned int ii = 0; ii < nImgHeight; ii++ ) {
        unsigned char* pFbAux = pFb + ( (nScreenWidth * (nTop + ii) ) + nLeft ) * 3;
        for( unsigned int jj = 0; jj < nImgWidth; jj++ ) {
            pFbAux[0] = pFbAux[1] = pFbAux[2] = *pImg++;
            //pFbAux[3] = 255;
            pFbAux += 3;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void SendLossyUC1SL(
        unsigned char*      pFb,
        unsigned int        nScreenWidth,
        unsigned char*      pImg,
        unsigned int        nImgHeight,
        unsigned int        nImgWidth,
        unsigned int        nTop
        )
{
    unsigned char* pFbAux = pFb + ( (nScreenWidth * nTop ) * 3 );
    for( unsigned int ii = 0; ii < nImgHeight; ii++ ) {
        for( unsigned int jj = 0; jj < nImgWidth; jj++ ) {
            pFbAux[0] = pFbAux[1] = pFbAux[2] = *pImg++;
            pFbAux += 3;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void SendLossyUC3(
        unsigned char*      pFb,
        unsigned int        nScreenWidth,
        unsigned char*      pImg,
        unsigned int        nImgHeight,
        unsigned int        nImgWidth,
        unsigned int        nTop,
        unsigned int        nLeft
        )
{
    for( unsigned int ii = 0; ii < nImgHeight; ii++ ) {
        unsigned char* pFbAux = pFb + ( (nScreenWidth * (nTop + ii) ) + nLeft ) * 3;
        memcpy( pFbAux, pImg, nImgWidth * 3 );
        pImg += nImgWidth * 3;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void SendLossyUC3SL(
        unsigned char*      pFb,
        unsigned int        nScreenWidth,
        unsigned char*      pImg,
        unsigned int        nImgHeight,
        unsigned int        nImgWidth,
        unsigned int        nTop
        )
{
    unsigned char* pFbAux = pFb + ( (nScreenWidth * nTop) * 3 );
    unsigned int nTotalBytes = 3 * nImgWidth * nImgHeight;
    memcpy( pFbAux, pImg, nTotalBytes );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void SendLosslessUC1(
        unsigned char*      pFb,
        unsigned int        nScreenWidth,
        unsigned char*      pImg,
        unsigned int        nImgHeight,
        unsigned int        nImgWidth,
        unsigned int        nTop,
        unsigned int        nLeft
        )
{
    unsigned int low_offset = nImgHeight * nScreenWidth * 3;

    for( unsigned int ii = 0; ii < nImgHeight; ii++ ) {
        unsigned char* pH = pFb + ( (nScreenWidth * (nTop + ii) ) + nLeft ) * 3;
        unsigned char* pL = pH + low_offset;
        for( unsigned int jj = 0; jj < nImgWidth; jj++ ) {
            unsigned char byte = *pImg++;
        
            unsigned char low = (0x0F & byte) << 4;
            unsigned char high = byte & 0xF0;
            
	        // pack high bits in first pixel
            pH[0] = pH[1] = pH[2] = high;
            pH += 3;

            // pack low bits in second pixel
            pL[0] = pL[1] = pL[2] = low; 
            pL += 3;
        }
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void SendDepth(
        unsigned char*      pFb,
        unsigned int        nScreenWidth,
        unsigned char*      pImg,
        unsigned int        nImgHeight,
        unsigned int        nImgWidth,
        unsigned int        nLoOrderTop,
        unsigned int        nHiOrderTop
        )
{
    unsigned char* pL = pFb + ( ( nScreenWidth * nLoOrderTop ) * 3 );
    unsigned char* pH = pFb + ( ( nScreenWidth * nHiOrderTop ) * 3 );
    for( unsigned int ii = 0; ii < nImgHeight; ii++ ) {
        for( unsigned int jj = 0; jj < nImgWidth; jj++ ) {
            unsigned char LoByte = *pImg++;
            unsigned char HiByte = *pImg++;

            pL[0] = pL[1] = pL[2] = LoByte;
            pL += 3;

            // pack high bit in first byte
            pH[0] = pH[1] = pH[2] = ( HiByte & 0x80 ? 255 : 0);
            pH += 3;

            pH[0] = pH[1] = pH[2] = ( HiByte & 0x40 ? 255 : 0 );
            pH += 3;

            pH[0] = pH[1] = pH[2] = ( HiByte & 0x20 ? 255 : 0 );
            pH += 3;

            pH[0] = pH[1] = pH[2] = ( HiByte & 0x10 ? 255 : 0 );
            pH += 3;

            pH[0] = pH[1] = pH[2] = ( HiByte & 0x08 ? 255 : 0 );
            pH += 3;

            pH[0] = pH[1] = pH[2] = ( HiByte & 0x04 ? 255 : 0 );
            pH += 3;

            pH[0] = pH[1] = pH[2] = ( HiByte & 0x02 ? 255 : 0 );
            pH += 3;

            pH[0] = pH[1] = pH[2] = ( HiByte & 0x01 ? 255 : 0 );
            pH += 3;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void SendID(
        unsigned char*      pFb,
        unsigned int        nScreenWidth,
        unsigned char       nID,
        unsigned int        nLineNum
        )
{
            unsigned char* pH = pFb + ( nScreenWidth * nLineNum ) * 3;
            
            pH[0] = pH[1] = pH[2] = ( nID & 0x80 ? 255 : 0);
            pH[3] = 255;
            pH += 4;

            pH[0] = pH[1] = pH[2] = ( nID & 0x40 ? 255 : 0 );
            pH[3] = 255;
            pH += 4;

            pH[0] = pH[1] = pH[2] = ( nID & 0x20 ? 255 : 0 );
            pH[3] = 255;
            pH += 4;

            pH[0] = pH[1] = pH[2] = ( nID & 0x10 ? 255 : 0 );
            pH[3] = 255;
            pH += 4;

            pH[0] = pH[1] = pH[2] = ( nID & 0x08 ? 255 : 0 );
            pH[3] = 255;
            pH += 4;

            pH[0] = pH[1] = pH[2] = ( nID & 0x04 ? 255 : 0 );
            pH[3] = 255;
            pH += 4;

            pH[0] = pH[1] = pH[2] = ( nID & 0x02 ? 255 : 0 );
            pH[3] = 255;
            pH += 4;

            pH[0] = pH[1] = pH[2] = ( nID & 0x01 ? 255 : 0 );
            pH[3] = 255;
            pH += 4;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    //--------------------------------------------------------------------------------------------------------
    // set up frame buffers
    int fbfd;
    if ((fbfd = open("/dev/fb0", O_RDWR)) == -1) {
        printf("Failed to open framebuffer.\n");
        exit(-1);
    }

    struct fb_fix_screeninfo finfo;
    if (ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo)) {
        printf("Error reading fixed information.\n");
        exit(-1);
    }

    struct fb_var_screeninfo vinfo;
    if (ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo)) {
        printf("Error reading variable information.\n");
        exit(-1);
    }

    // set video config
    vinfo.bits_per_pixel = 24;
    vinfo.red.offset    = 0;
    vinfo.red.length    = 8;
    vinfo.green.offset  = 8;
    vinfo.green.length  = 8;
    vinfo.blue.offset   = 16;
    vinfo.blue.length   = 8;
    vinfo.transp.offset = 0;
    vinfo.transp.length = 0;

    if (ioctl(fbfd, FBIOPUT_VSCREENINFO, &vinfo)) {
        printf("Error writing variable information.\n");
        exit(-1);
    }
    
    // figure out size of screen in pixels
    const unsigned int nScreenWidth = vinfo.xres;
    const unsigned int nScreenHeight = vinfo.yres;
    std::cout << "Screen Dimensions: " << nScreenWidth << " x " << nScreenHeight << std::endl;
    const long int nScreenSize = vinfo.xres * vinfo.yres * vinfo.bits_per_pixel / 8;
    printf("Screen total size is: %ld\n", nScreenSize);
    printf("Bits per pixel: %d\n", vinfo.bits_per_pixel);

    unsigned char* fbp;
    fbp = (unsigned char *) mmap(0, nScreenSize, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);
    if ((int) fbp == -1) {
        printf("Error mapping framebuffer to memory.\n");
        exit(-1);
    }

    // reset screen to black
    memset( fbp, 0, nScreenSize );
    sleep(1);

    /////////////////////////////////////////////////////////////////////////////////////

    GetPot clArgs(argc,argv);

    // init driver
    std::cout << "Initializing camera..." << std::endl;
    hal::Camera Cam( clArgs.follow("","-cam") );

    // container for images
    pb::ImageArray vImages;

    // N cameras, each w*h in dimension, greyscale
    const size_t nNumChannels = Cam.NumChannels();
    const size_t nImageWidth = Cam.Width();
    const size_t nImageHeight = Cam.Height();

    std::cout << "- Opening camera with " << nNumChannels << " channel(s)." << std::endl;
    for(size_t ii=0; ii<nNumChannels; ++ii) {
        std::cout << "  " << Cam.Width(ii) << "x" << Cam.Height(ii) << std::endl;
    }

    // variable to calculate frame rate
    double t = hal::Tic();
    unsigned int nFrames = 0;

    for( unsigned char ID = 0; ; ID++ ) {

        if( !Cam.Capture(vImages) ) {
            std::cout << "Error getting images." << std::endl;
        }

        // wait for vsync signal
        if( ioctl(fbfd, FBIO_WAITFORVSYNC, 0) ) {
          printf("VSync failed.\n");
        }
     
        //SendID( fbp, nScreenWidth, ID, 0 );
        SendLossyUC1( fbp, nScreenWidth, vImages[0].data(), nImageHeight, nImageWidth, 1 );
    	//SendDepth( fbp, 1280, vImages[1].Image.data, 240, 320, 80, 200 );

        //usleep( 1000000 / 30 );
        //sleep(1);

        nFrames++;
        double dTimeLapse = hal::Toc(t);
        if( dTimeLapse > 1.0 ){
            printf( "Framerate: %.2f\r", nFrames/dTimeLapse );
            fflush(stdout);
            nFrames  = 0;
            t = hal::Tic();
        }
    }
    return 0;
}
