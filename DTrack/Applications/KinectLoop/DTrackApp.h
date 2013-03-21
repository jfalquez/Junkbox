#ifndef _DTRACK_APP_
#define _DTRACK_APP_

#include <RPG/Utils/InitCam.h>
#include <Mvlpp/Mvl.h>
#include <DenseFrontEnd/DenseFrontEnd.h>

#include "Gui.h"

class DTrackApp
{
    public:

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        DTrackApp()
        {
            m_pFrontEnd = NULL;
            m_pMap = NULL;
            m_pTimer = NULL;
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool InitReset(
                int         argc,                       //< Input:
                char**      argv                        //< Input:
            )
        {
            // parse command line arguments
            GetPot clArgs( argc, argv );

            m_Cam.SetProperty("BufferSize", 20);

            // initialize camera
            if( !rpg::InitCam( m_Cam, clArgs ) ) {
                exit(0);
            }

            // capture check
            m_vImages.clear();
            m_Cam.Capture( m_vImages );

            // prepare images as expected by the FrontEnd
            _UnpackImages( m_vImages );

            if( m_pTimer ) {
                delete m_pTimer;
            }
            m_pTimer = new Timer;

            if( m_pMap ) {
                delete m_pMap;
            }
            m_pMap = new DenseMap;
            // load map from files IF user provided it!


            // get camera configuration
            std::string sSrcDir = m_Cam.GetProperty( "DataSourceDir", "." );

            std::string sCModFile;

            sCModFile = clArgs.follow( "cmod.xml", "-cmod" );
            std::string sGreyCModFilename = sSrcDir + "/" + sCModFile;

            sCModFile = clArgs.follow( "cmod_d.xml", "-dcmod" );
            std::string sDepthCModFilename = sSrcDir + "/" + sCModFile;

            // TODO load files here so that GUI is aware of intrinsics or something
            // perhaps store it in the map??


            if( m_pFrontEnd ) {
                delete m_pFrontEnd;
            }
            m_pFrontEnd = new DenseFrontEnd( m_vImages[0].width(), m_vImages[0].height() );
            return m_pFrontEnd->Init( sGreyCModFilename, sDepthCModFilename, m_vImages, m_pMap, m_pTimer );
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void StepOnce( Gui& rGui )
        {
            if( m_Cam.Capture( m_vImages ) ) {
                _UnpackImages( m_vImages );
                if( m_pFrontEnd->Iterate( m_vImages ) == false) {
                    std::cerr << "critical warning: something went wrong during the last iteration." << std::endl;
                    rGui.SetState( PAUSED );
                }

                if( m_pFrontEnd->TrackingState() == eTrackingBad || m_pFrontEnd->TrackingState() == eTrackingLoopClosure ) {
                    rGui.SetState( PAUSED );
                }
            } else {
                rGui.SetState( PAUSED );
            }
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void UpdateGui( Gui& rGui )
        {
            rGui.CopyMapChanges( *m_pMap );

            rGui.UpdateImages( m_vImages[0].Image );

//            rGui.UpdateTimer( m_pTimer->GetWindowSize(), m_pTimer->GetNames(3), m_pTimer->GetTimes(3) );

        }



    private:

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void _UnpackImages(
                CamImages&          vImages    //< Input/Output
            )
        {
            /* */

            // this converts images from the kinect to the expected format of DTrack
            cv::Mat Tmp;

            // convert RGB to GREYSCALE
            cv::cvtColor( vImages[0].Image, vImages[0].Image, CV_RGB2GRAY, 1 );

            // check if second image is provided, if so we assume it is the DEPTH image
            if( vImages.size() > 1 ) {
                // convert 16U to 32FC1
                vImages[1].Image.convertTo( Tmp, CV_32FC1 );

                // convert mm to m
                Tmp = Tmp / 1000;

                vImages[1].Image = Tmp;
            }

            /* */
        }


    /////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private:

        CameraDevice                m_Cam;          // camera handler
        CamImages                   m_vImages;      // camera images
        Eigen::Matrix3d             m_Kg;           // intensity (ie. greyscale) camera's intrinsics
        Eigen::Matrix3d             m_Kd;           // depth camera's intrinsics
        Eigen::Matrix4d             m_Tgd;          // depth camera's pose w.r.t. the greyscale camera

        DenseFrontEnd*              m_pFrontEnd;
        DenseMap*                   m_pMap;
        Timer*                      m_pTimer;
};

#endif
