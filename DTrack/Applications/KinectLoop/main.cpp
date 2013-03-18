#include "DTrackApp.h"

/////////////////////////////////////////////////////////////////////////////
void DTrackThread( Gui& gui, int argc, char** argv)
{
    DTrackApp app;
    if( !app.InitReset( argc, argv ) ) {
        gui.SetState( QUIT );   // this will notify the GUI to die
    } else {
        gui.SetState( PAUSED );
    }

    while( gui.State != QUIT ) {

        if( gui.State == RESETTING ) {
            app.InitReset( argc, argv );
            gui.SetState( RESET_COMPLETE );
            usleep( 10000 );
        }
        if( gui.State == PLAYING ) {
            app.StepOnce( gui );
        }
        if( gui.State == STEPPING ) {
            app.StepOnce( gui );
            gui.SetState( PAUSED );
        }

        app.UpdateGui( gui );

        if( gui.State == PAUSED ) {
            usleep(1000000 / 30);
        }
    }
}


/////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    // create GUI window
    Gui gui("KinectLoop", 1280, 800);
    gui.Init();

    // start tracker thread
    boost::thread TT( DTrackThread, boost::ref(gui), argc, argv);

    /*
    DTrackApp app;
    if( !app.InitReset( argc, argv ) ) {
        std::cerr << "error: a problem occured while initializing app." << std::endl;
    }

    std::cout << "Sleeping..." << std::endl;
    sleep(1);

    app.StepOnce( gui );
    */

    // start gui thread
    gui.Run();

    return 0;
}
