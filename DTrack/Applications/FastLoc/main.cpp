#include <thread>

#include "DTrackApp.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DTrackThread( Gui& gui, int argc, char** argv)
{
    try {
        DTrackApp app;
        if( app.InitReset( argc, argv ) == false ) {
            gui.SetState( QUIT );   // this will notify the GUI to die
        }
        app.UpdateGui( gui );

        while( gui.State != QUIT ) {

            if( gui.State == RESETTING ) {
                if( app.InitReset( argc, argv ) == false ) {
                    gui.SetState( QUIT );
                } else {
                    gui.SetState( RESET_COMPLETE );
                    usleep( 10000 );
                }
            }

            app.StepOnce( gui );
            app.UpdateGui( gui );

        }
    } catch( std::exception ) {
        gui.State = QUIT;   // this will notify the GUI to die
        std::cerr << "error: an exception occured in DTrack thread. " << std::endl;
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    // create GUI window
    Gui gui("DTrack", 1280, 800);
    gui.Init();

    // start tracker thread
    std::thread TT( DTrackThread, std::ref(gui), argc, argv);

    // start gui thread
    gui.Run();

    // wait for thread to join
    TT.join();

    return 0;
}
