#include <stdio.h>
#include <stdlib.h>

#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>

#include <opencv.hpp>

#include <RPG/Utils/GetPot>
#include <RPG/Utils/InitCam.h>

using namespace std;

int main(int argc, char** argv)
{
    // parse command line arguments
    GetPot clArgs( argc, argv );

    // number of valid images to capture
    unsigned int nNumCaps  = clArgs.follow( 15, "-num" );

    int nBoardWidth     = 9;
    int nBoardHeight    = 6;
    int nBoardTotal     = nBoardWidth * nBoardHeight;           // total enclosed corners on the board
    CvSize nBoardSize = cvSize( nBoardWidth, nBoardHeight );

    // initialize camera
    CameraDevice                        Cam;
    std::vector< rpg::ImageWrapper >    vImages;
    cv::Mat                             ColorImg;
    cv::Mat                             GreyImg;
    cv::Mat                             Snapshot;

    rpg::InitCam( Cam, clArgs );

    // initial capture to obtain image dimensions
    Cam.Capture( vImages );

    const unsigned int nImgWidth = vImages[0].Image.cols;
    const unsigned int nImgHeight = vImages[0].Image.rows;


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // create a GUI window
    pangolin::CreateGlutWindowAndBind( "Camera Calibration", 1200, 600 );
    SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();

    // create panel
    pangolin::View& glPanel = pangolin::CreatePanel("ui");
    glPanel.SetBounds( 0, 1.0, 0.0, pangolin::Attach::Pix(250) );

    pangolin::Var<string>           ui_Status( "ui.Status" );
    pangolin::Var<unsigned int>     ui_nNumSnapshots( "ui.Number of Snapshots" );
    pangolin::Var<double>           ui_dCalError( "ui.Calibration Error", 0 );
    pangolin::Var<bool>             ui_btnCalibrate( "ui.Calibrate", false, false );

    pangolin::View& guiContainer = pangolin::CreateDisplay()
            .SetBounds(0,1,pangolin::Attach::Pix(250),1)
            .SetLayout(pangolin::LayoutEqual);

    // display images
    SceneGraph::ImageView glLiveImg( true, true );
    glLiveImg.SetAspect( (double)nImgWidth / nImgHeight );

    SceneGraph::ImageView glSnapshot( true, true );
    glSnapshot.SetAspect( (double)nImgWidth / nImgHeight );

    guiContainer.AddDisplay( glLiveImg );
    guiContainer.AddDisplay( glSnapshot );


    bool            bSnapshot      = false;


    // keyboard callbacks
    pangolin::RegisterKeyPressCallback(' ', [&bSnapshot](){ bSnapshot = true; });



    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    unsigned int nNumSnapshots = 0;
    vector< vector< cv::Point3f > > vObjectPts;
    vector< vector< cv::Point2f > > vImagePts;

    vector< cv::Point3f > vObj;
    for(int ii = 0; ii < nBoardTotal; ii++) {
        vObj.push_back( cv::Point3f( ii / nBoardWidth, ii % nBoardWidth, 0.0f));
    }

    cv::Mat             Intrinsics = cv::Mat( 3, 3, CV_32FC1 );
    cv::Mat             Distortion;
    vector< cv::Mat >   vRot;
    vector< cv::Mat >   vTrans;

    while( !pangolin::ShouldQuit() ) {

        Cam.Capture( vImages );

        ColorImg = vImages[0].Image;

        // convert to greyscale
        cv::cvtColor( ColorImg, GreyImg, CV_RGB2GRAY );

        // find for chessboard pattern
        vector< cv::Point2f > vCorners; // this will be filled by the detected corners

        bool bPatternFound = cv::findChessboardCorners( GreyImg, nBoardSize, vCorners,
                cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK );

        if( bPatternFound ) {
            ui_Status = "OK";

            cv::cornerSubPix( GreyImg, vCorners, cv::Size(11, 11), cv::Size(-1, -1),
                           cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1) );
            cv::drawChessboardCorners( ColorImg, nBoardSize, cv::Mat(vCorners), true );

            // only snapshot if pattern is found
            if( bSnapshot ) {
                nNumSnapshots++;
                ui_nNumSnapshots = nNumSnapshots;
                Snapshot = ColorImg;
                vImagePts.push_back( vCorners );
                vObjectPts.push_back( vObj );
                glSnapshot.SetImage( Snapshot.data, nImgWidth, nImgHeight, GL_RGB8, GL_RGB, GL_UNSIGNED_BYTE );
            }
        } else {
            ui_Status = "--";
        }

        // reset snapshot GUI request
        bSnapshot = false;


        if( pangolin::Pushed( ui_btnCalibrate ) ) {
            cout << "Calibrating..." << endl;
            double error = cv::calibrateCamera( vObjectPts, vImagePts, ColorImg.size(), Intrinsics, Distortion, vRot, vTrans );
            ui_dCalError = error;
            cout << "... Done!" << endl << endl;
            cout << "Intrinsics = "<< endl << " "  << Intrinsics << endl << endl;
        }


        //----------------------------------------------------------------------------------------

        // clear whole screen
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

        // show images
        glLiveImg.SetImage( ColorImg.data, nImgWidth, nImgHeight, GL_RGB8, GL_RGB, GL_UNSIGNED_BYTE );

        pangolin::FinishGlutFrame();
    }


    /*
    //Loop while successful captures equals total snapshots
    //Successful captures implies when all the enclosed corners are detected from a snapshot

    while(nSuccesses < nNumCaps)
    {
        if((frame++ % frame_step) == 0)									//Skip frames
        {
            //Find chessboard corners:
            int found = cvFindChessboardCorners(image, nBoardSize, vCorners, &nCornerCount,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );

            cvCvtColor(image, gray_image, CV_BGR2GRAY);										//Get Subpixel accuracy on those corners
            cvFindCornerSubPix(gray_image, vCorners, nCornerCount, cvSize(11,11),cvSize(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

            cvDrawChessboardCorners(image, nBoardSize, vCorners, nCornerCount, found);			//Draw it

            // If we got a good board, add it to our data
            if( nCornerCount == nBoardTotal )
            {
                cvShowImage( "Snapshot", image );										//show in color if we did collect the image
                step = nSuccesses*nBoardTotal;
                for( int i=step, j=0; j<nBoardTotal; ++i,++j ) {
                CV_MAT_ELEM(*image_points, float,i,0) = vCorners[j].x;
                CV_MAT_ELEM(*image_points, float,i,1) = vCorners[j].y;
                CV_MAT_ELEM(*object_points,float,i,0) = (float) j/nBoardWidth;
                CV_MAT_ELEM(*object_points,float,i,1) = (float) (j%nBoardWidth);
                CV_MAT_ELEM(*object_points,float,i,2) = 0.0f;
            }
            CV_MAT_ELEM(*point_counts, int,nSuccesses,0) = nBoardTotal;
            nSuccesses++;
            printf("\r%d successful Snapshots out of %d collected.",nSuccesses,nNumCaps);
            }
            else
            cvShowImage( "Snapshot", gray_image );										//Show Gray if we didn't collect the image
        }


        //Handle pause/unpause and ESC
        int c = cvWaitKey(15);
        if(c == 'p')
        {
            c = 0;
            while(c != 'p' && c != 27)
            {
                c = cvWaitKey(250);
            }
        }
        if(c == 27)
            return 0;

        image = cvQueryFrame( capture );								//Get next image
        cvShowImage("Raw Video", image);
    }

    //End WHILE loop with enough successful captures

    cvDestroyWindow("Snapshot");

    printf("\n\n *** Calbrating the camera now...\n");

    //Allocate matrices according to successful number of captures
    CvMat* object_points2  = cvCreateMat(nSuccesses*nBoardTotal,3,CV_32FC1);
    CvMat* image_points2   = cvCreateMat(nSuccesses*nBoardTotal,2,CV_32FC1);
    CvMat* point_counts2   = cvCreateMat(nSuccesses,1,CV_32SC1);

    //Tranfer the points to matrices
    for(int i = 0; i<nSuccesses*nBoardTotal; ++i)
    {
      CV_MAT_ELEM( *image_points2, float, i, 0)  =	CV_MAT_ELEM( *image_points, float, i, 0);
      CV_MAT_ELEM( *image_points2, float,i,1)    =	CV_MAT_ELEM( *image_points, float, i, 1);
      CV_MAT_ELEM(*object_points2, float, i, 0)  =  CV_MAT_ELEM( *object_points, float, i, 0) ;
      CV_MAT_ELEM( *object_points2, float, i, 1) =  CV_MAT_ELEM( *object_points, float, i, 1) ;
      CV_MAT_ELEM( *object_points2, float, i, 2) =  CV_MAT_ELEM( *object_points, float, i, 2) ;
    }

    for(int i=0; i<nSuccesses; ++i)
    {
        CV_MAT_ELEM( *point_counts2, int, i, 0) =  CV_MAT_ELEM( *point_counts, int, i, 0);			//These are all the same number
    }
    cvReleaseMat(&object_points);
    cvReleaseMat(&image_points);
    cvReleaseMat(&point_counts);

    // Initialize the intrinsic matrix with both the two focal lengths in a ratio of 1.0

    CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ) = 1.0f;
    CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ) = 1.0f;

    //Calibrate the camera
    //_____________________________________________________________________________________

    cvCalibrateCamera2(object_points2, image_points2, point_counts2,  cvGetSize( image ), intrinsic_matrix, distortion_coeffs, NULL, NULL,0 );
                                                                                                                                        //CV_CALIB_FIX_ASPECT_RATIO

    //_____________________________________________________________________________________

    //Save values to file
    printf(" *** Calibration Done!\n\n");
    printf("Storing Intrinsics.xml and Distortions.xml files...\n");
    cvSave("Intrinsics.xml",intrinsic_matrix);
    cvSave("Distortion.xml",distortion_coeffs);
    printf("Files saved.\n\n");

    printf("Starting corrected display....\n");

    //Sample: load the matrices from the file
    CvMat *intrinsic = (CvMat*)cvLoad("Intrinsics.xml");
    CvMat *distortion = (CvMat*)cvLoad("Distortion.xml");

    // Build the undistort map used for all subsequent frames.

    IplImage* mapx = cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1 );
    IplImage* mapy = cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1 );
    cvInitUndistortMap(intrinsic,distortion,mapx,mapy);

    // Run the camera to the screen, showing the raw and the undistorted image.

    cvNamedWindow( "Undistort" );
    while(image)
    {
        IplImage *t = cvCloneImage(image);
        cvShowImage( "Raw Video", image );			// Show raw image
        cvRemap( t, image, mapx, mapy );			// Undistort image
        cvReleaseImage(&t);
        cvShowImage("Undistort", image);			// Show corrected image

        //Handle pause/unpause and ESC
        int c = cvWaitKey(15);
        if(c == 'p')
        {
            c = 0;
            while(c != 'p' && c != 27)
            {
                c = cvWaitKey(250);
            }
        }
        if(c == 27)
            break;

        image = cvQueryFrame( capture );
    }
    */

    return 0;
}
