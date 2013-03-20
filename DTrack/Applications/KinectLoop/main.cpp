#include "DTrackApp.h"

/////////////////////////////////////////////////////////////////////////////
void DTrackThread( Gui& gui, int argc, char** argv)
{
    try {
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
    } catch( std::exception ) {
        gui.State = QUIT;   // this will notify the GUI to die
        std::cerr << "error: an exception occured in DTrack thread. " << std::endl;
    }
}


/////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    // create GUI window
    Gui gui("KinectLoop", 1280, 800);
    gui.Init();

    /// two threads
    /* */
    // start tracker thread
    boost::thread TT( DTrackThread, boost::ref(gui), argc, argv);

    // start gui thread
    gui.Run();

    // wait for thread to join
    TT.join();
    /* */

    /// single thread
    /*
    DTrackApp app;
    if( !app.InitReset( argc, argv ) ) {
        std::cerr << "error: a problem occured while initializing app." << std::endl;
    }

    app.UpdateGui( gui );

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    pangolin::FinishGlutFrame();

    while( !pangolin::ShouldQuit() ) {

        std::cout << ".";
        sleep(1);

        app.StepOnce( gui );

        app.UpdateGui( gui );

        if( gui.m_bMapDirty ) {
            // the map has changed update it before rendering again
            gui.m_pRenderMap->CopyMapChanges( *(gui.m_pChangesBufferMap) );
            gui.m_bMapDirty = false;
        }

        app.UpdateGui( gui );

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        pangolin::FinishGlutFrame();

        usleep(1000000 / 120);
    }
    /* */


    return 0;
}
