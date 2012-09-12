#include <stdlib.h>
#include <string>
#include <vector>

#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>
#include <flycapture/FlyCapture2.h>


using namespace std;
using namespace FlyCapture2;

#define CAM2 0

typedef std::tuple<Image, Image>        ImagePair;		// image 1 and image 2
std::deque<ImagePair>                   g_qImages;



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
void SaveAviHelper(
    std::vector<Image>& vecImages, 
    std::string aviFileName, 
    float frameRate)
{
    Error error;
    AVIRecorder aviRecorder;

    // Open the AVI file for appending images

    AVIOption option;
    option.frameRate = frameRate;
    error = aviRecorder.AVIOpen(aviFileName.c_str(), &option);

    if (error != PGRERROR_OK) {
        PrintError(error);
        return;
    }    

    printf( "\nAppending %ld images to AVI file: %s ... \n", vecImages.size(), aviFileName.c_str() );
    for(unsigned int imageCnt = 0; imageCnt < vecImages.size(); imageCnt++) {
        // Append the image to AVI file
        error = aviRecorder.AVIAppend(&vecImages[imageCnt]);
        if (error != PGRERROR_OK) {
            PrintError(error);
            continue;
        }

        printf("Appended image %d...\n", imageCnt); 
    }

    // Close the AVI file
    error = aviRecorder.AVIClose( );
    CheckError(error);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int /*argc*/, char** /*argv*/)
{
    const unsigned int nImgWidth = 1280;
    const unsigned int nImgHeight = 1024;

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

    // look for camera 1
    error = BusMgr.GetCameraFromIndex(0, &GUID);
    CheckError(error);

    // connect to a camera 1
    error = Cam1.Connect(&GUID);
    CheckError(error);

    CameraInfo  CamInfo;

    // print camera 1 info
    printf("Camera 1\n");
    error = Cam1.GetCameraInfo( &CamInfo );
    CheckError(error);

    PrintCameraInfo(&CamInfo);

#if CAM2
    // look for camera 2
    error = BusMgr.GetCameraFromIndex(1, &GUID);
    CheckError(error);

    // connect to a camera 2
    error = Cam2.Connect(&GUID);
    CheckError(error);

    // print camera 2 info
    printf("Camera 2\n");
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
    SceneGraph::ImageView			 glLeftImg;
    SceneGraph::ImageView			 glRightImg;
    glLeftImg.SetBounds( 0.0, 1.0, 0.0, 0.5, (double)nImgWidth / nImgHeight );
    glRightImg.SetBounds( 0.0, 1.0, 0.5, 1.0, (double)nImgWidth / nImgHeight );

    // Add our views as children to the base container.
    glBaseView.AddDisplay( glLeftImg );
    glBaseView.AddDisplay( glRightImg );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Start capturing images
    printf( "Starting capture... \n" );
    error = Cam1.StartCapture();
    CheckError(error);

#if CAM2
    error = Cam2.StartCapture();
    CheckError(error);
#endif

    Image LeftImg, RightImg;


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    while( !pangolin::ShouldQuit() ) {

        error = Cam1.RetrieveBuffer( &LeftImg );

        if( error != PGRERROR_OK ) {
            printf("Error grabbing camera 1 image.\n");
        }

#if CAM2
        error = Cam2.RetrieveBuffer( &RightImg );

        if( error != PGRERROR_OK ) {
            printf("Error grabbing camera 1 image.\n");
        }
#endif


        //-------------------------------------------------------------------------------------------------------------


        // clear whole screen
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

        // show left image
        glLeftImg.SetImage( LeftImg.GetData(), nImgWidth, nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_UNSIGNED_BYTE );

#if CAM2
        // show right image
        glRightImg.SetImage( RightImg.GetData(), nImgWidth, nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_UNSIGNED_BYTE );
#endif

        pangolin::FinishGlutFrame();
    }

    // stop capture
    Cam1.StopCapture();

    /*
    // Start capturing images
    printf( "Starting capture... \n" );
    error = Cam1.StartCapture();
    CheckError(error);


    const unsigned int k_numImages = 10;
    std::vector<Image> vecImages;
    vecImages.resize(k_numImages);

    // Grab images
    Image rawImage;
    ImageMetadata Meta1, Meta2;
    for ( unsigned int imageCnt=0; imageCnt < k_numImages; imageCnt++ )
    {
        error = Cam1.RetrieveBuffer(&rawImage);

//        Meta1 = rawImage.GetMetadata();


        if (error != PGRERROR_OK)
        {
            printf("Error grabbing image %u\n", imageCnt);
            continue;
        }
        else
        {
            printf("Grabbed image %u\n", imageCnt);
        }

        vecImages[imageCnt].DeepCopy(&rawImage);
    }

    // Stop capturing images
    printf( "Stopping capture... \n" );
    error = Cam1.StopCapture();
    CheckError(error);

    // Check if the camera supports the FRAME_RATE property
    printf( "Detecting frame rate from camera... \n" );
    PropertyInfo propInfo;
    propInfo.type = FRAME_RATE;
    error = Cam1.GetPropertyInfo( &propInfo );
    CheckError(error);

    float frameRateToUse = 60.0f;
    if ( propInfo.present == true )
    {
        // Get the frame rate
        Property prop;
        prop.type = FRAME_RATE;
        error = Cam1.GetProperty( &prop );
        CheckError(error);

        // Set the frame rate.
        // Note that the actual recording frame rate may be slower,
        // depending on the bus speed and disk writing speed.
        frameRateToUse = prop.absValue;
    }

    printf("Using frame rate of %3.1f\n", frameRateToUse);

    char aviFileName[512] = {0};

    sprintf(aviFileName, "SaveImageToAviEx-Uncompressed-%u", camInfo.serialNumber);
    SaveAviHelper(vecImages, aviFileName, frameRateToUse);

    // Disconnect the cameras
    error = Cam1.Disconnect();
    CheckError(error);
*/

    return 0;
}
