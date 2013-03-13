#include "DTrackApp.h"

/////////////////////////////////////////////////////////////////////////////
void DTrackThread( Gui& gui, int argc, char** argv)
{
    DTrackApp app;
    app.InitReset( argc, argv );
    app.UpdateGui( gui );
    gui.SetState( PAUSED );

    while( gui.State != QUIT ) {

        if( gui.State == RESETTING ){
            app.InitReset( argc, argv );
            gui.SetState( RESET_COMPLETE );
            usleep( 10000 );
            app.UpdateGui( gui );
        }
        if( gui.State == PLAYING ){
            app.StepOnce( gui );
            app.UpdateGui( gui );
        }
        if( gui.State == STEPPING ){
            app.StepOnce( gui );
            app.UpdateGui( gui );
            gui.SetState( PAUSED );
        }
        if( gui.State == PAUSED) {
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

    // start gui thread
    gui.Run();

    return 0;
}
