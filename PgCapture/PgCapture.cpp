#include <stdlib.h>
#include <string>
#include <vector>
#include <iomanip>
#include <mutex>
#include <thread>
#include <condition_variable>

#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>
#include <flycapture/FlyCapture2.h>

#include "TicToc.h"
#include "PropertyMap.h"


using namespace std;
using namespace FlyCapture2;

#define CAM2 0


// image variables and buffer
struct ImgWrapper {
    Image           image;
    PropertyMap     map;
};

typedef std::tuple< ImgWrapper, ImgWrapper >        ImagePair;              // image 1 and image 2
std::deque< ImagePair >                             g_qImages;
unsigned int                                        g_nMaxBufferSize = 3;
unsigned int                                        g_nDroppedFrames = 0;
unsigned int                                        g_nCount = 0;


// synch stuff
volatile bool                                       g_bKillThread = false;
volatile bool                                       g_bCapture = false;
std::condition_variable                             g_Condition;
std::mutex                                          g_Mutex, g_ConditionMutex;
std::thread                                         g_ThreadSave;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PrintError( Error error )
{
    error.PrintErrorTrace();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void CheckError( Error err )
{
    if( err != PGRERROR_OK ) {
       PrintError(err);
       exit(-1);
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PrintCameraInfo( CameraInfo* pCamInfo )
{
    printf(
        "\n*** CAMERA INFORMATION ***\n"
        "Serial number - %u\n"
        "Camera model - %s\n"
        "Camera vendor - %s\n"
        "Sensor - %s\n"
        "Resolution - %s\n"
        "Firmware version - %s\n"
        "Firmware build time - %s\n\n",
        pCamInfo->serialNumber,
        pCamInfo->modelName,
        pCamInfo->vendorName,
        pCamInfo->sensorInfo,
        pCamInfo->sensorResolution,
        pCamInfo->firmwareVersion,
        pCamInfo->firmwareBuildTime );
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GoCapture()
{
    g_bCapture = g_bCapture ? false : true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DequeueAndSaveImage()
{
    printf( "Dequeue Thread started... \n" );

    double                  dPrevTimeStampWrite = Tic();
    double                  dSumTimeWrite = 0;

    Error                   error;
    AVIRecorder             aviRecorder;
    const std::string       sFileName = "PgCapture.avi";


    // Open the AVI file for appending images

    AVIOption AviOption;
    AviOption.frameRate = 60.0;
    error = aviRecorder.AVIOpen( sFileName.c_str(), &AviOption );
    CheckError(error);

    std::unique_lock<std::mutex> cond( g_ConditionMutex );

    ImagePair ImagePair;
    while( g_bKillThread == false ) {
        while( g_bCapture ) {
            if( g_qImages.empty() ) {
                //std::cout << "Empty" << std::endl;
                // Wait for signal
                g_Condition.wait( cond );
            }

    //        std::cout << "Not empty" << std::endl;
            while( !g_qImages.empty() ) {

                // Lock when popping from dequeue
                {
                    std::lock_guard<std::mutex> mutex( g_Mutex );

                    ImagePair = g_qImages.front();

                    g_qImages.pop_front();
                }

                std::cout << "Buffer: " << g_qImages.size() << " Image #:";

                dSumTimeWrite += Toc( dPrevTimeStampWrite );

                cout << setw(10) << "FPS: " << 1 / Toc( dPrevTimeStampWrite );

                dPrevTimeStampWrite = Tic();

                cout << setw(10) << "FPS: " << double(g_nCount) / dSumTimeWrite << endl;

                // append the image to AVI file
                error = aviRecorder.AVIAppend( &(std::get<0>(ImagePair).image) );
                if( error != PGRERROR_OK ) {
                    PrintError(error);
                    continue;
                }
#if CAM2
                error = aviRecorder.AVIAppend( &(std::get<1>(ImagePair).image) );
                if( error != PGRERROR_OK ) {
                    PrintError(error);
                    continue;
                }
#endif
                // save timestamp to text file
            }
        }
    }
    printf( "\n... Dequeue Thread died!!!\n" );

    // Close the AVI file
    error = aviRecorder.AVIClose( );
    CheckError(error);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int /*argc*/, char** /*argv*/)
{
    VideoMode           DefaultMode      = VIDEOMODE_1280x960Y16;
    FrameRate           DefaultFrameRate = FRAMERATE_60;
    const unsigned int  nImgWidth        = 1280;
    const unsigned int  nImgHeight       = 960;

    Error error;

    BusManager      BusMgr;
    unsigned int    nNumCams;

    error = BusMgr.GetNumOfCameras( &nNumCams );
    CheckError(error);

    if( nNumCams != 1 ) {
       printf( "Two cameras are required to run this program.\n" );
       return -1;
    }

    Camera      Cam1;
    Camera      Cam2;
    PGRGuid     GUID;
    CameraInfo  CamInfo;

    // look for camera 1
    error = BusMgr.GetCameraFromIndex(0, &GUID);
    CheckError(error);

    // connect to camera 1
    error = Cam1.Connect(&GUID);
    CheckError(error);

    // set video mode and framerate
    error = Cam1.SetVideoModeAndFrameRate( DefaultMode, DefaultFrameRate );
    CheckError(error);

    // prepare trigger
    TriggerMode Trigger;

    // external trigger is disabled
    Trigger.onOff = false;
    Trigger.mode = 0;
    Trigger.parameter = 0;
    Trigger.source = 0;

    // prepare strobe
    StrobeControl Strobe;
    StrobeInfo StrInfo;

    StrInfo.source = 1;
    Cam1.GetStrobeInfo(&StrInfo);

    printf(" On Off %d\n", StrInfo.onOffSupported);
    printf(" Max %f\n", StrInfo.maxValue);
    printf(" Min %f\n", StrInfo.minValue);
    printf(" Source %d\n", StrInfo.source);
    printf(" Present %d\n", StrInfo.present);

    Cam1.SetGPIOPinDirection( 1, 1 );
//    exit(0);

    // set pin 2 as strobe
    Strobe.onOff = true;
    Strobe.source = 1;
    Strobe.delay = 0;
    Strobe.duration = 0;
    Strobe.polarity = 0;

//    error = Cam2.SetStrobe( &Strobe );
    CheckError(error);

    // print camera 1 info
    printf("------------------------- Camera 1 -------------------------\n");
    error = Cam1.GetCameraInfo( &CamInfo );
    CheckError(error);

    PrintCameraInfo(&CamInfo);

#if CAM2
    // look for camera 2
    error = BusMgr.GetCameraFromIndex(1, &GUID);
    CheckError(error);

    // connect to camera 2
    error = Cam2.Connect(&GUID);
    CheckError(error);

    // set video mode and framerate
    error = Cam2.SetVideoModeAndFrameRate( DefaultMode, DefaultFrameRate );
    CheckError(error);

    // set camera to trigger mode 0
    // default GPIO pin for "trigger in" is 1
    Trigger.onOff = true;
    Trigger.mode = 0;
    Trigger.parameter = 0;
    Trigger.source = 0;

    error = Cam2.SetTriggerMode( &Trigger );
    CheckError(error);

    // print camera 2 info
    printf("------------------------- Camera 2 -------------------------\n");
    error = Cam2.GetCameraInfo( &CamInfo );
    CheckError(error);

    PrintCameraInfo(&CamInfo);
#endif

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Create OpenGL window in single line thanks to GLUT
    pangolin::CreateGlutWindowAndBind( "Point Grey Logger", 1024, 420 );
    SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();

    // Scenegraph to hold GLObjects and relative transformations
    SceneGraph::GLSceneGraph    glGraph;

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState glState( pangolin::ProjectionMatrix( 640, 480, 420, 420, 320, 240, 0.1, 1000 ),
                                      pangolin::ModelViewLookAt( -6, 0, -30, 1, 0, 0, pangolin::AxisNegZ ) );

    // Pangolin abstracts the OpenGL viewport as a View.
    // Here we get a reference to the default 'base' view.
    pangolin::View& glBaseView = pangolin::DisplayBase();

    // display images
    SceneGraph::ImageView			 glLeftImg(true, false);
    SceneGraph::ImageView			 glRightImg(true, false);
    glLeftImg.SetBounds( 0.0, 1.0, 0.0, 0.5, (double)nImgWidth / nImgHeight );
    glRightImg.SetBounds( 0.0, 1.0, 0.5, 1.0, (double)nImgWidth / nImgHeight );

    // Add our views as children to the base container.
    glBaseView.AddDisplay( glLeftImg );
    glBaseView.AddDisplay( glRightImg );

    // register key callbacks
    pangolin::RegisterKeyPressCallback( 'c', GoCapture );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // launch de-queue thread
    g_ThreadSave = std::thread( &DequeueAndSaveImage );

    // start capturing images
    printf( "Starting capture... \n" );
    error = Cam1.StartCapture();
    CheckError(error);

#if CAM2
    error = Cam2.StartCapture();
    CheckError(error);
#endif

    // image storage
    Image Image1, Image2;

    // variable to calculate frame rate
    double          dTic = Tic();
    unsigned int    nFrames = 0;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Main Loop
    //
    while( !pangolin::ShouldQuit() ) {

        error = Cam1.RetrieveBuffer( &Image1 );

        if( error != PGRERROR_OK ) {
            printf("Error grabbing camera 1 image.\n");
        }

#if CAM2
        error = Cam2.RetrieveBuffer( &RightImg );

        if( error != PGRERROR_OK ) {
            printf("Error grabbing camera 2 image.\n");
        }
#endif

        //-------------------------------------------------------------------------------------------------------------


        if( g_bCapture == true ) {

            if( g_qImages.size() > g_nMaxBufferSize ) {
                g_nDroppedFrames++;

                std::cout << "Dropping frame (num. dropped: " << g_nDroppedFrames << ")." << std::endl;
                {    // Lock when popping from dequeue
                    std::lock_guard< std::mutex > mutex( g_Mutex );

                    g_qImages.pop_back();
                    g_nCount--;
                }
                continue;
            } else {
                if( false ) {
                    cerr << "ERROR: empty image buffer returned from camera." << endl;
                    continue;
                }

                // increment image counter
                g_nCount++;

                // get timestamp
                double dTimeStamp = Tic ();

                ImgWrapper Image1w;
                ImgWrapper Image2w;

                Image1w.image.DeepCopy( &Image1 );
                Image2w.image.DeepCopy( &Image2 );

                Image1w.map.SetProperty( "TimeStamp", dTimeStamp );

                {
                    std::lock_guard< std::mutex > mutex( g_Mutex );

                    g_qImages.push_back( ImagePair( Image1w, Image2w ) );
                }

                g_Condition.notify_one();
            }
        }

        //-------------------------------------------------------------------------------------------------------------

        nFrames++;
        double dTimeLapse = Toc(dTic);
        if( dTimeLapse > 1.0 ) {
            printf( "Framerate: %.2f\r", nFrames/dTimeLapse );
            fflush(stdout);
            nFrames = 0;
            dTic = Tic();
        }

        //-------------------------------------------------------------------------------------------------------------


        // clear whole screen
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

        // show left image
        glLeftImg.SetImage( Image1.GetData(), nImgWidth, nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_UNSIGNED_SHORT );

#if CAM2
        // show right image
        glRightImg.SetImage( RightImg.GetData(), nImgWidth, nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_UNSIGNED_SHORT );
#endif

        pangolin::FinishGlutFrame();
    }

    // stop capture thread
    g_bKillThread = true;
    g_Condition.notify_one();
    if( g_ThreadSave.joinable() ) {
        g_ThreadSave.join();
    }

    // stop capture
    printf( "Stopping capture... \n" );
    Cam1.StopCapture();

#if CAM2
    Cam2.StopCapture();
#endif

    printf( "... Done!\n" );

    return 0;
}
