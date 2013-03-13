#include "DTrackApp.h"

/////////////////////////////////////////////////////////////////////////////
void DTrackThread( Gui& gui, int argc, char** argv)
{
    SlamApp app;
    app.InitReset( argc, argv );
    app.UpdateGui( gui );
    gui.SetState( PAUSED );

    while( gui.state != QUIT ) {

        if( gui.state == RESETTING ){
            app.InitReset( argc, argv );
            gui.SetState( RESET_COMPLETE );
            usleep( 10000 );
            app.UpdateGui( gui );
        }
        if( gui.state == PLAYING ){
            app.StepOnce( gui );
        }
        if( gui.state == STEPPING ){
            app.StepOnce( gui );
            gui.SetState( PAUSED );
        }
        if( gui.state == PAUSED) {
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
    boost::thread tracker( DTrackThread, boost::ref(gui), argc, argv);

    // start gui thread
    gui.Run();

    return 0;
}


