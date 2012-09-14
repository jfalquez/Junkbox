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
    data.gyro = data.gyro * 9.81;

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
    pangolin::Var<unsigned int> nImgIdx("ui.Image ID", 0);
    pangolin::Var<unsigned int> nBlur("ui.Blur",1,0,5);

    // create a view container
    pangolin::View& guiContainer = pangolin::CreateDisplay()
            .SetBounds(0,1,pangolin::Attach::Pix(300),1)
            .SetLayout(pangolin::LayoutEqual);

    // add 3d view to container
    guiContainer.AddDisplay(glView3D);

    // draw grid and path on 3D window
    SceneGraph::GLGrid  glGrid;
    GLPath              glPath;
    SceneGraph::GLAxis  glAxis;
    glGraph.AddChild( &glGrid );
    glGraph.AddChild( &glPath );
    glGraph.AddChild( &glAxis );

    // keyboard callbacks

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

    ViconData_t     CurVicon, NextVicon;
    ImuData_t       CurImu, NextImu;
    double          CurStereo;

    CurVicon = ReadVicon( fVicon );
    CurImu = ReadImu( fImu );

    while( !pangolin::ShouldQuit() ) {

        // read stereo timestamp
        // push IMU poses and Vicon poses until they are about to skip stereo timestamp
        //

        //SeFu.RegisterImuPose();
        if( fVicon.eof( ) == false ) {
            glPath.PushPose( CurVicon.pose );
            CurVicon = ReadVicon( fVicon );
        }

        if( fVicon.eof( ) || fImu.eof( ) ) {
            cerr << "Ran out of IMU and/or Vicon readings!" << endl;
            break;
        }


        ///------------------------------------------------------------------------------------------------------------


        // update and render stuff
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
