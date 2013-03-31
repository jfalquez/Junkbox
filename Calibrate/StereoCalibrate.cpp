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
    std::cout << "DISCLAIMER: -idev option is NOT available in this program." << std::endl;

    // parse command line arguments
    GetPot clArgs( argc, argv );

    // using the default OpenCV checkerboard pattern
    int nBoardWidth     = 9;
    int nBoardHeight    = 6;
    int nBoardTotal     = nBoardWidth * nBoardHeight;           // total enclosed corners on the board
    float fSquareSize   = 0.023;
    CvSize nBoardSize = cvSize( nBoardWidth, nBoardHeight );

    // camera stuff
    CameraDevice                        Cam1;   // external RGB camera
    CameraDevice                        Cam2;   // Kinect's IR image
    std::vector< rpg::ImageWrapper >    vImages1;
    std::vector< rpg::ImageWrapper >    vImages2;

    // the two images we are calibrating
    cv::Mat                             ColorImg1;
    cv::Mat                             GreyImg1;
    cv::Mat                             Snapshot1;

    cv::Mat                             ColorImg2;
    cv::Mat                             GreyImg2;
    cv::Mat                             Snapshot2;


    // initialize camera
//    Cam1.SetProperty( "Rectify", true );
//    Cam1.SetProperty( "ForceGreyscale", true );
//    Cam1.InitDriver( "Bumblebee2" );
    Cam1.InitDriver( "FireFly" );


    Cam2.SetProperty( "GetRGB", false );
    Cam2.SetProperty( "GetDepth", false );
    Cam2.SetProperty( "GetIr", true );
    Cam2.InitDriver( "Kinect" );


    // initial capture to obtain image dimensions
    Cam1.Capture( vImages1 );

    const unsigned int nImgWidth1 = vImages1[0].Image.cols;
    const unsigned int nImgHeight1 = vImages1[0].Image.rows;
    std::cout << "Image #1 Dimensions: " << nImgWidth1 << " x " << nImgHeight1 << std::endl;

    Cam2.Capture( vImages2 );

//    const unsigned int nImgWidth2 = nImgWidth1;
//    const unsigned int nImgHeight2 = nImgHeight1;
    const unsigned int nImgWidth2 = vImages2[0].Image.cols;
    const unsigned int nImgHeight2 = vImages2[0].Image.rows;
    std::cout << "Image #2 Dimensions: " << nImgWidth2 << " x " << nImgHeight2 << std::endl;


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // create a GUI window
    pangolin::CreateGlutWindowAndBind( "Stereo Camera Calibration", 1200, 600 );
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
    SceneGraph::ImageView glLiveImg1( true, true );
    glLiveImg1.SetAspect( (double)nImgWidth1 / nImgHeight1 );

    SceneGraph::ImageView glSnapshot1( true, true );
    glSnapshot1.SetAspect( (double)nImgWidth1 / nImgHeight1 );

    SceneGraph::ImageView glLiveImg2( true, true );
    glLiveImg2.SetAspect( (double)nImgWidth2 / nImgHeight2 );

    SceneGraph::ImageView glSnapshot2( true, true );
    glSnapshot2.SetAspect( (double)nImgWidth2 / nImgHeight2 );

    // add images in container
    guiContainer.AddDisplay( glLiveImg1 );
    guiContainer.AddDisplay( glSnapshot1 );
    guiContainer.AddDisplay( glLiveImg2 );
    guiContainer.AddDisplay( glSnapshot2 );


    bool            bSnapshot      = false;


    // keyboard callbacks
    pangolin::RegisterKeyPressCallback(' ', [&bSnapshot](){ bSnapshot = true; });
    pangolin::RegisterKeyPressCallback('c', [&ui_btnCalibrate](){ ui_btnCalibrate = true; });


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    unsigned int nNumSnapshots = 0;
    vector< vector< cv::Point3f > > vObjectPts;
    vector< vector< cv::Point2f > > vImagePts1;
    vector< vector< cv::Point2f > > vImagePts2;

    vector< cv::Point3f > vObj;
    for(int ii = 0; ii < nBoardTotal; ii++) {
        vObj.push_back( cv::Point3f( ii / nBoardWidth, ii % nBoardWidth, 0.0f));
    }

//    cv::Mat             Intrinsics1 = cv::Mat( 3, 3, CV_32FC1 );
    // K hint for Cam1
    cv::Mat Intrinsics1 = (cv::Mat_<float>(3,3) <<    655.0681058933573, 0, 329.3888800064832,
                                                      0, 651.5601207003715, 249.7271121691255,
                                                      0, 0, 1);
//    cv::Mat             Distortion1;
    cv::Mat Distortion1 = (cv::Mat_<float>(1,5) <<   -0.4309355351200019, 0.2749971654145275, 0.002517876773074356, -0.0003738676467441764, -0.1696187437955576);


//    cv::Mat             Intrinsics2 = cv::Mat( 3, 3, CV_32FC1 );
    // K hint for Cam2
    cv::Mat Intrinsics2 = (cv::Mat_<float>(3,3) << 579.3257324220499, 0, 319.2756869108347,
                                                   0, 576.0530308640929, 238.578345871265,
                                                   0, 0, 1);
    cv::Mat             Distortion2;


    cv::Mat   vRot;
    cv::Mat   vTrans;
    cv::Mat   vE;
    cv::Mat   vF;


    while( !pangolin::ShouldQuit() ) {

        /// camera #1
        Cam1.Capture( vImages1 );

        // which one is it?
//        ColorImg1 = vImages1[0].Image;
//        cv::resize( vImages1[0].Image, ColorImg1, cv::Size(0,0), 0.5, 0.5 );
        cv::undistort( vImages1[0].Image, GreyImg1, Intrinsics1, Distortion1 );
//        GreyImg1 = vImages1[0].Image;
//        cv::resize( vImages1[0].Image, GreyImg1, cv::Size(0,0), 0.5, 0.5 );

        //----- convert greyscale to RGB
        cv::cvtColor( GreyImg1, ColorImg1, CV_GRAY2RGB );

        //----- convert RGB to greyscale
//        cv::cvtColor( ColorImg, GreyImg, CV_RGB2GRAY );



        /// camera #2
        Cam2.Capture( vImages2 );

        //----- convert InfraRed to greyscale
        vImages2[0].Image.convertTo( GreyImg2, CV_8UC1 );

        // which one is it?
//        ColorImg2 = vImages2[0].Image;
//        cv::resize( vImages2[0].Image, ColorImg2, cv::Size(0,0), 0.5, 0.5 );
//        GreyImg2 = vImages2[0].Image;
//        cv::resize( vImages2[0].Image, GreyImg2, cv::Size(0,0), 0.5, 0.5 );

        //----- convert greyscale to RGB
        cv::cvtColor( GreyImg2, ColorImg2, CV_GRAY2RGB );

        //----- convert RGB to greyscale
//        cv::cvtColor( ColorImg2, GreyImg2, CV_RGB2GRAY );


        bool bPatternFound = false;

        // find for checkerboard pattern
        vector< cv::Point2f > vCorners1; // this will be filled by the detected corners
        vector< cv::Point2f > vCorners2; // this will be filled by the detected corners

        bool bPattern1Found = cv::findChessboardCorners( GreyImg1, nBoardSize, vCorners1,
                cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK );

        bool bPattern2Found = cv::findChessboardCorners( GreyImg2, nBoardSize, vCorners2,
                cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK );

        if( bPattern1Found && bPattern2Found ) {
            ui_Status = "OK";

            // only snapshot if pattern is found
            if( bSnapshot ) {
                cv::cornerSubPix( GreyImg1, vCorners1, cv::Size(11, 11), cv::Size(-1, -1),
                               cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1) );
                cv::drawChessboardCorners( ColorImg1, nBoardSize, cv::Mat(vCorners1), true );

                cv::cornerSubPix( GreyImg2, vCorners2, cv::Size(11, 11), cv::Size(-1, -1),
                               cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1) );
                cv::drawChessboardCorners( ColorImg2, nBoardSize, cv::Mat(vCorners2), true );

                nNumSnapshots++;
                ui_nNumSnapshots = nNumSnapshots;
                vObjectPts.push_back( vObj );
                Snapshot1 = ColorImg1;
                vImagePts1.push_back( vCorners1 );
                glSnapshot1.SetImage( Snapshot1.data, nImgWidth1, nImgHeight1, GL_RGB8, GL_RGB, GL_UNSIGNED_BYTE );
                Snapshot2 = ColorImg2;
                vImagePts2.push_back( vCorners2 );
                glSnapshot2.SetImage( Snapshot2.data, nImgWidth2, nImgHeight2, GL_RGB8, GL_RGB, GL_UNSIGNED_BYTE );
            }
        } else {
            ui_Status = "--";
        }

        // reset snapshot GUI request
        bSnapshot = false;


        if( pangolin::Pushed( ui_btnCalibrate ) ) {
            cout << "Calibrating..." << endl;
            double error = cv::stereoCalibrate( vObjectPts, vImagePts1, vImagePts2,
                                                Intrinsics1, Distortion1, Intrinsics2, Distortion2,
                                                ColorImg1.size(), vRot, vTrans, vE, vF,
                                                cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 1e-6),
                                                cv::CALIB_FIX_INTRINSIC );
            ui_dCalError = error;
            cout << "... Done!" << endl << endl;
            cout << "Intrinsics1 = "<< endl << " "  << Intrinsics1 << endl << endl;
            cout << "Intrinsics2 = "<< endl << " "  << Intrinsics2 << endl << endl;
            cout << "----------------------------------" << endl;
            cout << "Translation = "<< endl << " "  << vTrans * fSquareSize << endl << endl;
            cout << "Rotation = "<< endl << " "  << vRot << endl << endl;
//             [-0.009325362943876125; -0.04070539569298045; -0.0268853648137168
        }


        //----------------------------------------------------------------------------------------

        // clear whole screen
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

        // show images
        if( bPatternFound ) {
            glLiveImg1.SetImage( ColorImg1.data, nImgWidth1, nImgHeight1, GL_RGB8, GL_RGB, GL_UNSIGNED_BYTE );
            glLiveImg2.SetImage( ColorImg2.data, nImgWidth2, nImgHeight2, GL_RGB8, GL_RGB, GL_UNSIGNED_BYTE );
        } else {
            glLiveImg1.SetImage( GreyImg1.data, nImgWidth1, nImgHeight1, GL_INTENSITY, GL_LUMINANCE, GL_UNSIGNED_BYTE );
            glLiveImg2.SetImage( GreyImg2.data, nImgWidth2, nImgHeight2, GL_INTENSITY, GL_LUMINANCE, GL_UNSIGNED_BYTE );
        }

        pangolin::FinishGlutFrame();
    }

    return 0;
}
