#include <dc1394/dc1394.h>
#include <dc1394/conversions.h>

#include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


void _cleanup_and_exit(dc1394camera_t *pCam) {
    dc1394_video_set_transmission( pCam, DC1394_OFF );
    dc1394_capture_stop( pCam );
    dc1394_camera_free( pCam );
    exit(-1);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// da main
int main( int argc, char** argv )
{

    dc1394_t*           pBus;
    dc1394camera_t*     pCam;
    dc1394error_t       e;
    unsigned int        nImageWidth = 512;
    unsigned int        nImageHeight = 384;
    cv::Mat             Img;


    // here we connect to the firefly and see if it's alive
    pBus = dc1394_new();

    dc1394camera_list_t* pCameraList = NULL;
    e = dc1394_camera_enumerate(pBus, &pCameraList);

    if (pCameraList->num == 0) {
        printf("No cameras found!\n");
        exit(-1);
    }

    for( int ii = 0; ii < (int)pCameraList->num; ii++) {
        pCam = dc1394_camera_new( pBus, pCameraList->ids[ii].guid );
        printf("Model %s\n", pCam->model );
    }

    // free the camera list
    dc1394_camera_free_list( pCameraList );
    printf("Using camera with GUID %llu\n", pCam->guid);

    // set ISO speed
    e = dc1394_video_set_iso_speed( pCam, DC1394_ISO_SPEED_400 );
    DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(pCam), "Could not set iso speed.");

    // set format 7 mode 0
    dc1394video_mode_t nVideoMode = DC1394_VIDEO_MODE_FORMAT7_0;
    e = dc1394_video_set_mode( pCam, nVideoMode );
    DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(pCam), "Could not set video mode.");

    // set image position
    e = dc1394_format7_set_image_position( pCam, nVideoMode, 56, 0 );
    DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(pCam), "Could not set image position.");

    // set image size
    uint nMaxWidth, nMaxHeight;
    e = dc1394_format7_get_max_image_size( pCam, nVideoMode, &nMaxWidth, &nMaxHeight );
    std::cout << "Max Width: " << nMaxWidth << " -- Max Height: " << nMaxHeight << std::endl;
    e = dc1394_format7_set_image_size( pCam, nVideoMode, 512, 384 );
    DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(pCam), "Could not set image size.");

    // set color coding
    dc1394color_coding_t nCoding = DC1394_COLOR_CODING_MONO8;
    e = dc1394_format7_set_color_coding( pCam, nVideoMode, nCoding );
    DC1394_ERR_CLN_RTN(e,_cleanup_and_exit(pCam),"Could not set color coding.");

    // ROI - framerate, width, offset, etc.
    e = dc1394_format7_set_roi( pCam, nVideoMode, nCoding, 4095, 56, 0, nImageWidth, nImageHeight );
    DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(pCam), "Could not set ROI.");

    // set up camera
    int nNumDMAChannels = 2;
    e = dc1394_capture_setup( pCam, nNumDMAChannels, DC1394_CAPTURE_FLAGS_DEFAULT );
    DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(pCam), "Could not setup camera. Make sure that the video mode and framerate are supported by your camera.");

    // print camera features
    dc1394featureset_t vFeatures;
    e = dc1394_feature_get_all( pCam, &vFeatures );
    if (e != DC1394_SUCCESS) {
        dc1394_log_warning("Could not get feature set");
    } else {
        dc1394_feature_print_all( &vFeatures, stdout );
    }

    //
//    dc1394trigger_mode_t nTrigMode;
//    e = dc1394_external_trigger_set_mode( pCam, nTrigMode );
//    DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(pCam), "Error setting trigger sources.");

    // set trigger to manual
//    e = dc1394_feature_set_mode( pCam, DC1394_FEATURE_TRIGGER, DC1394_FEATURE_MODE_MANUAL );
//    DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(pCam), "Error setting trigger mode.");

   // set software trigger option as source
    dc1394trigger_source_t nTrigSource = DC1394_TRIGGER_SOURCE_SOFTWARE;
    e = dc1394_external_trigger_set_source( pCam, nTrigSource );
    DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(pCam), "Error setting trigger source.");

    e = dc1394_external_trigger_set_power( pCam, DC1394_ON );
    DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(pCam), "Error setting external trigger.");


    // initiate transmission
    e = dc1394_video_set_transmission( pCam, DC1394_ON );
    DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(pCam), "Could not start camera iso transmission.");



    //  capture frame variables
    dc1394video_frame_t * pFrame;
    dc1394capture_policy_t nPolicy = DC1394_CAPTURE_POLICY_WAIT;


    // create GUI windows
	cv::namedWindow( "Image", CV_WINDOW_AUTOSIZE );

	while(1) {
        // trigger camera
        e = dc1394_software_trigger_set_power( pCam, DC1394_ON );
        DC1394_ERR_CLN_RTN(e,_cleanup_and_exit(pCam),"Error triggering the camera.");

        // capture
        e = dc1394_capture_dequeue(pCam, nPolicy, &pFrame);
        DC1394_ERR_CLN_RTN(e,_cleanup_and_exit(pCam),"Could not capture a frame.");

        Img = cv::Mat(384, 512, CV_8UC1, pFrame->image);

		cv::imshow("Image", Img);

		char c;
		c = cv::waitKey(2);
		if (c == 27) break;

        // release
        e = dc1394_capture_enqueue( pCam, pFrame );
        DC1394_ERR_CLN_RTN(e,_cleanup_and_exit(pCam),"Could not release frame.");
	}
    _cleanup_and_exit(pCam);
	return 0;
}