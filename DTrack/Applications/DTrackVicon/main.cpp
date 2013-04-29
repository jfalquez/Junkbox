#include <thread>

#include "DTrackApp.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DTrackThread( Gui& gui, int argc, char** argv)
{
    try {
        DTrackApp app;
        if( app.InitReset( argc, argv ) == false ) {
            gui.SetState( QUIT );   // this will notify the GUI to die
        } else {
            gui.SetState( PAUSED );
        }

        while( gui.State != QUIT ) {

            if( gui.State == RESETTING ) {
                if( app.InitReset( argc, argv ) == false ) {
                    gui.SetState( QUIT );
                } else {
                    gui.SetState( RESET_COMPLETE );
                    usleep( 10000 );
                }
            }
            if( gui.State == PLAYING ) {
                app.StepOnce( gui );
            }
            if( gui.State == STEPPING ) {
                app.StepOnce( gui );
                gui.SetState( PAUSED );
            }
            if( gui.State == VICON_ALIGN ) {
                app.CallViconAlign();
                gui.SetState( PAUSED );
            }

            app.UpdateGui( gui );

            if( gui.State == PAUSED ) {
                usleep(1000000 / 30);
            }
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
