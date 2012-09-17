#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>

#include "GetPot"
#include "SensorFusion.h"
#include "GLPath.h"

using namespace std;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct ViconData_t {
    double              system_time;
    double              vicon_time;
    string              label;
    Eigen::Vector6d     pose;
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct ImuData_t {
    double              system_time;
    Eigen::Vector3d     accel;
    Eigen::Vector3d     gyro;
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ViconData_t ReadVicon( ifstream& File )
{
    ViconData_t data;

    File >> data.system_time >> data.vicon_time >> data.label >> data.pose(0) >> data.pose(1)
             >> data.pose(2) >> data.pose(3) >> data.pose(4) >> data.pose(5);

    // convert pose to robotics frame
    Eigen::Matrix4d T = mvl::Cart2T( data.pose );
    Eigen::Matrix3d Rvicon_robot;
    Rvicon_robot <<  1,  0,  0,
                     0, -1,  0,
                     0,  0, -1;
    T.block<3,3>(0,0) = Rvicon_robot * T.block<3,3>(0,0) * Rvicon_robot;
    T(1,3) = - T(1,3);
    T(2,3) = - T(2,3);

    data.pose = mvl::T2Cart( T );

    return data;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ImuData_t ReadImu( ifstream& File )
{
    ImuData_t data;

    File >> data.system_time >> data.accel(0) >> data.accel(1) >> data.accel(2)
            >> data.gyro(0) >> data.gyro(1) >> data.gyro(2);

    // acceleration is given in Gs, therefore convert to m/s2
    data.accel = data.accel * IMU_GRAVITY_CONST;

    data.accel(0) = -data.accel(0);
    data.accel(1) = -data.accel(1);
    data.gyro(0) = -data.gyro(0);
    data.gyro(1) = -data.gyro(1);

//    data.accel(0) = 0;
//    data.accel(1) = 0;
//    data.accel(2) = -IMU_GRAVITY_CONST;
//    data.gyro(0) = 0;
//    data.gyro(1) = 0;
//    data.gyro.setZero();

    return data;

}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
    GetPot cl( argc, argv );

    // for file reader
    std::string sDataDir    = cl.follow( ".", 1, "-sdir" );
    std::string sViconFile  = cl.follow( "Vicon.txt", 1, "-vicon" );
    std::string sImuFile    = cl.follow( "IMU.txt", 1, "-imu" );
    std::string sOutputFile = cl.follow( "Output.txt", 1, "-out" );
    int         nFilterSize = cl.follow( 10, 1, "-fwin" );
    double      dStart      = cl.follow( 0.0, 1, "-stime" );

    // create a GUI window
    pangolin::CreateGlutWindowAndBind("FuseIV", 1200, 600);
    glewInit();
    SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState glState( pangolin::ProjectionMatrix( 640, 480, 420, 420, 320, 240, 0.1, 1000 ),
                                         pangolin::ModelViewLookAt( 0, -6, -10, 0, 1, 0, pangolin::AxisNegZ ) );

    // Scenegraph to hold GLObjects and relative transformations
    SceneGraph::GLSceneGraph glGraph;

    // We define a new view which will reside within the container.
    pangolin::View glView3D;
    glView3D.SetAspect(640.0f/480.0f);

    // We set the views location on screen and add a handler
    glView3D.SetHandler( new SceneGraph::HandlerSceneGraph( glGraph, glState, pangolin::AxisNegZ ) );
    glView3D.SetDrawFunction( SceneGraph::ActivateDrawFunctor( glGraph, glState ) );

    // create a side panel
    pangolin::CreatePanel("ui").SetBounds(0,1,0,pangolin::Attach::Pix(300));
    pangolin::Var<float>        ui_fTime("ui.Time", 0.0);
    pangolin::Var<float>        ui_fVelX("ui.Velocity X", 0.0);
    pangolin::Var<float>        ui_fVelY("ui.Velocity Y", 0.0);
    pangolin::Var<float>        ui_fVelZ("ui.Velocity Z", 0.0);
    pangolin::Var<float>        ui_fAccelX("ui.Acceleration X", 0.0);
    pangolin::Var<float>        ui_fAccelY("ui.Acceleration Y", 0.0);
    pangolin::Var<float>        ui_fAccelZ("ui.Acceleration Z", 0.0);

    // create a view container
    pangolin::View& guiContainer = pangolin::CreateDisplay()
            .SetBounds(0,1,pangolin::Attach::Pix(300),1)
            .SetLayout(pangolin::LayoutEqual);

    // add 3d view to container
    guiContainer.AddDisplay(glView3D);

    // draw grid and path on 3D window
    SceneGraph::GLGrid  glGrid;
    GLPath              glViconPath;
    GLPath              glFilteredPath;
    SceneGraph::GLAxis  glAxis;
    glGraph.AddChild( &glGrid );
    glGraph.AddChild( &glViconPath );
    glGraph.AddChild( &glFilteredPath );
    glFilteredPath.SetLineColor( 1.0, 0.0, 0.0 );
    glGraph.AddChild( &glAxis );

    // keyboard callbacks
    bool guiStep = false;
    bool guiRunning = false;

    pangolin::RegisterKeyPressCallback(' ',[&guiRunning](){ guiRunning = !guiRunning; });
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + GLUT_KEY_RIGHT,[&guiStep](){ guiStep = true; });

    // data files
    ifstream fVicon, fImu;
    string sFileURL = sDataDir + "/" + sViconFile;
    fVicon.open( sFileURL.c_str() );
    sFileURL = sDataDir + "/" + sImuFile;
    fImu.open( sFileURL.c_str() );
    ofstream fOut;
    fOut.open( sOutputFile.c_str() );

    if( fVicon.is_open() == false ) {
        cerr << "Vicon file not found." << endl;
        exit(-1);
    }

    if( fImu.is_open() == false ) {
        cerr << "IMU file not found." << endl;
        exit(-1);
    }

    if( fOut.is_open() == false ) {
        cerr << "Output file could not be created." << endl;
        exit(-1);
    }

    // instantiate a SensorFusion
    SensorFusion    SeFu( nFilterSize );

    // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Main Loop
    //

    PoseParameter   CurPose;

    ViconData_t     CurVicon, NextVicon;
    ImuData_t       CurImu, NextImu;
    double          CurStereo;
    double          dStartTime;

    // initialize sensor fusion
    CurVicon = ReadVicon( fVicon );
    CurImu = ReadImu( fImu );

    // skip Vicon frames if a start time was passed through param list
    dStartTime = CurVicon.system_time + dStart;
    while( CurVicon.system_time <  dStartTime ) {
        CurVicon = ReadVicon( fVicon );
    }

    // print some info
    printf( "First Vicon data @ %10.5f\n", CurVicon.system_time );
    printf( "First IMU data @ %10.5f\n", CurImu.system_time );
    cout << "======================================================================================" << endl;


    // this alignment is probably BROKEN?

    // align Vicon to first IMU time
    while( CurVicon.system_time < CurImu.system_time ) {
        //printf( "Discarding Vicon data @ %10.5f\n", CurVicon.system_time );
        CurVicon = ReadVicon( fVicon );
    }

    cout << "======================================================================================" << endl;
    printf( "Current Vicon data @ %10.5f\n", CurVicon.system_time );
    printf( "Current IMU data @ %10.5f\n", CurImu.system_time );
    cout << "======================================================================================" << endl;

    // align IMU to Vicon's time
    while( CurImu.system_time < CurVicon.system_time ) {
        //printf( "Discarding IMU data @ %10.5f\n", CurImu.system_time );
        CurImu = ReadImu( fImu );
    }

    dStartTime = CurVicon.system_time;
    SeFu.ResetCurrentPose( CurVicon.pose, Eigen::Vector3d::Zero() , Eigen::Vector2d::Zero() );
    SeFu.RegisterGlobalPose( CurVicon.pose, CurVicon.system_time - dStartTime );

    while( !pangolin::ShouldQuit() ) {

        if( guiRunning || pangolin::Pushed(guiStep) ) {

            // read new vicon pose
            CurVicon = ReadVicon( fVicon );
            if( fVicon.eof( ) ) {
                cerr << "Ran out Vicon readings!" << endl;
                break;
            }

            // push IMU readings between previous pose and new pose
            while( CurImu.system_time < CurVicon.system_time ) {
                SeFu.RegisterImuPose( CurImu.accel, CurImu.gyro, CurImu.system_time - dStartTime );
                cout << SeFu._GetGravityVector(SeFu.GetCurrentPose().m_dG).transpose() << endl;
                CurImu = ReadImu( fImu );
                if( fImu.eof( ) ) {
                    cerr << "Ran out of IMU readings!" << endl;
                    break;
                }
            }

            // now that all previous IMU data has been pushed, push in the new global pose
            SeFu.RegisterGlobalPose( CurVicon.pose, CurVicon.system_time - dStartTime );

            // get filtered pose back
            CurPose = SeFu.GetCurrentPose();

            // write filtered pose to file
            fOut << CurPose.m_dPose.transpose() << endl;

            // update gui poses
            glViconPath.PushPose( CurVicon.pose );
            glFilteredPath.PushPose( CurPose.m_dPose );
        }

        ///------------------------------------------------------------------------------------------------------------

        // update gui variables
        ui_fTime = CurVicon.system_time - dStartTime;

        ui_fVelX = CurPose.m_dV(0);
        ui_fVelY = CurPose.m_dV(1);
        ui_fVelZ = CurPose.m_dV(2);

        ui_fAccelX = CurImu.accel(0) + CurPose.m_dG(0);
        ui_fAccelY = CurImu.accel(1) + CurPose.m_dG(1);
        ui_fAccelZ = CurImu.accel(2) + CurPose.m_dG(2);

        // render stuff
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
        glColor4f( 1, 1, 1, 1);

        pangolin::FinishGlutFrame();

    }

    // close data files
    fVicon.close( );
    fImu.close( );
    fOut.close( );

    return 0;
}
