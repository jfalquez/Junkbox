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

    // using the default OpenCV checkerboard pattern
    int nCircleWidth     = 4;
    int nCircleHeight    = 11;
    int nGridTotal     = nCircleWidth * nCircleHeight;           // total enclosed corners on the board
    CvSize nGridSize = cvSize( nCircleWidth, nCircleHeight );

    // initialize camera
    CameraDevice                        Cam;
    std::vector< rpg::ImageWrapper >    vImages;
    cv::Mat                             ColorImg;
    cv::Mat                             GreyImg;
    cv::Mat                             Snapshot;

    if( rpg::InitCam( Cam, clArgs ) == false ) {
        exit(1);
    }

    // initial capture to obtain image dimensions
    Cam.Capture( vImages );

    const unsigned int nImgWidth = vImages[0].Image.cols;
    const unsigned int nImgHeight = vImages[0].Image.rows;
    std::cout << "Image Dimensions: " << nImgWidth << " x " << nImgHeight << std::endl;


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
    pangolin::RegisterKeyPressCallback('c', [&ui_btnCalibrate](){ ui_btnCalibrate = true; });



    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    unsigned int nNumSnapshots = 0;
    vector< vector< cv::Point3f > > vObjectPts;
    vector< vector< cv::Point2f > > vImagePts;

    vector< cv::Point3f > vObj;
    for(int ii = 0; ii < nGridTotal; ii++) {
        vObj.push_back( cv::Point3f( ii / nCircleWidth, ii % nCircleWidth, 0.0f));
    }

    cv::Mat             Intrinsics = cv::Mat( 3, 3, CV_32FC1 );
    cv::Mat             Distortion;
    vector< cv::Mat >   vRot;
    vector< cv::Mat >   vTrans;

    while( !pangolin::ShouldQuit() ) {

        Cam.Capture( vImages );

        //----- convert InfraRed to greyscale
//        vImages[0].Image.convertTo( GreyImg, CV_8UC1 );

        // which one is it?
        ColorImg = vImages[0].Image;
//        cv::resize( vImages[0].Image, ColorImg, cv::Size(0,0), 0.5, 0.5 );
//        GreyImg = vImages[0].Image;
//        cv::resize( vImages[0].Image, GreyImg, cv::Size(0,0), 0.5, 0.5 );

        //----- convert greyscale to RGB
//        cv::cvtColor( GreyImg, ColorImg, CV_GRAY2RGB );

        //----- convert RGB to greyscale
        cv::cvtColor( ColorImg, GreyImg, CV_RGB2GRAY );

        //----- convert InfraRed to greyscale
//        ColorImg.convertTo( GreyImg, CV_8UC1 );


        // find for chessboard pattern
        vector< cv::Point2f > vCenters;     // this will be filled by the detector

        bool bPatternFound = cv::findCirclesGrid( GreyImg, nGridSize, vCenters, cv::CALIB_CB_ASYMMETRIC_GRID );

        if( bPatternFound ) {
            ui_Status = "OK";

            // only snapshot if pattern is found
            if( bSnapshot ) {

                cv::drawChessboardCorners( ColorImg, nGridSize, cv::Mat(vCenters), true );

                nNumSnapshots++;
                ui_nNumSnapshots = nNumSnapshots;
                Snapshot = ColorImg;
                vImagePts.push_back( vCenters );
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
            int Flags = 0;
//            int Flags = cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_SAME_FOCAL_LENGTH | cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2
//                    | cv::CALIB_FIX_K3| cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5 | cv::CALIB_FIX_K6;
            cv::TermCriteria Criteria = cv::TermCriteria( cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON );
            double error = cv::calibrateCamera( vObjectPts, vImagePts, ColorImg.size(), Intrinsics,
                                                Distortion, vRot, vTrans, Flags, Criteria );
            ui_dCalError = error;
            cout << "... Done! ( Error: " << error << " )" << endl << endl;
            cout << "Distortion = "<< endl << " "  << Distortion << endl << endl;
            cout << "Intrinsics = "<< endl << " "  << Intrinsics << endl << endl;
        }


        //----------------------------------------------------------------------------------------


        // clear whole screen
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

        // show images
        if( bPatternFound ) {
            glLiveImg.SetImage( ColorImg.data, nImgWidth, nImgHeight, GL_RGB8, GL_RGB, GL_UNSIGNED_BYTE );
        } else {
            glLiveImg.SetImage( GreyImg.data, nImgWidth, nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_UNSIGNED_BYTE );
        }

        pangolin::FinishGlutFrame();
    }

    return 0;
}
