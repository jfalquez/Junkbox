#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>

#include "GetPot"
#include "SensorFusion.h"

using namespace std;


int main(int argc, char** argv)
{
    GetPot cl( argc, argv );

    // for file reader
    std::string sViconFile  = cl.follow( "Vicon.txt", 1, "-vicon" );
    std::string sImuFile    = cl.follow( "IMU.txt", 1, "-imu" );
    std::string sOutputFile = cl.follow( "Output.txt", 1, "-out" );
    int         nFilterSize = cl.follow( 10, 1, "-fwin" );

    // create a GUI window
    pangolin::CreateGlutWindowAndBind("FuseIV", 800, 600);
    glewInit();
    SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState glState( pangolin::ProjectionMatrix( 640, 480, 420, 420, 320, 240, 0.1, 1000 ),
                                         pangolin::ModelViewLookAt( -6, 0, -10, 1, 0, 0, pangolin::AxisNegZ ) );

    // Scenegraph to hold GLObjects and relative transformations
    SceneGraph::GLSceneGraph glGraph;

    // We define a new view which will reside within the container.
    pangolin::View glView3D;

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

    // draw grid on 3D window
    SceneGraph::GLGrid glGrid;
    glGraph.AddChild( &glGrid );

    // keyboard callbacks

    // data files
    ifstream fVicon, fImu;
    fVicon.open( sViconFile.c_str() );
    fImu.open( sImuFile.c_str() );
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


            // read pose
            pFile >> ReadPose(0) >> ReadPose(1) >> ReadPose(2) >> ReadPose(3) >> ReadPose(4) >> ReadPose(5);

            if( pFile.eof( ) ) {
                break;
            }



    // instantiate a SensorFusion
    SensorFusion    SeFu( nFilterSize );

    // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Main Loop
    //
    while( !pangolin::ShouldQuit() ) {

        // read


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
