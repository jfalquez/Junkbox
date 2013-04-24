#ifndef _DTRACK_APP_
#define _DTRACK_APP_

#include <RPG/Utils/InitCam.h>
#include <Mvlpp/Mvl.h>
#include <DenseFrontEnd/FastLocalizer.h>
#include <DenseBackEnd/DenseBackEnd.h>

#include "Gui.h"

class DTrackApp
{
    public:

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        DTrackApp()
        {
            m_pLocalizer = NULL;
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

            // initialize camera
            if( !rpg::InitCam( m_Cam, clArgs ) ) {
                return false;
            }

            // capture check
            m_vImages.clear();
            m_Cam.Capture( m_vImages );

            //----- get extra parameters of our own making
            m_bConvertGrey = clArgs.search( "-rgb-to-grey" );
            m_bRectify = clArgs.search( "-rectify" );
            m_bResize = clArgs.search( "-resize" );


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
            // load map from files if user provided it!
            std::string sMap = clArgs.follow( "", "-map" );
            if( sMap.empty() == false ) {
                if( m_pMap->ImportMap( sMap ) == true ) {
                    m_pMap->UpdateInternalPathFull();
                } else {
                    std::cerr << "error: Could not load map '" << sMap << "'." << std::endl;
                }
            }

            // get camera configuration
            std::string sSrcDir = m_Cam.GetProperty( "DataSourceDir", "." );

            std::string sCModFile;

            sCModFile = clArgs.follow( "cmod.xml", "-cmod" );
            std::string sGreyCModFilename = sSrcDir + "/" + sCModFile;

            sCModFile = clArgs.follow( "cmod_d.xml", "-dcmod" );
            std::string sDepthCModFilename = sSrcDir + "/" + sCModFile;

            if (m_pMap->LoadCameraModels( sGreyCModFilename, sDepthCModFilename ) == false ) {
                std::cerr << "abort: There was a problem loading the camera model files!" << std::endl;
                return false;
            }

            if( m_pLocalizer ) {
                delete m_pLocalizer;
            }
            m_pLocalizer = new FastLocalizer( m_vImages[0].width(), m_vImages[0].height() );
            if( m_pLocalizer->Init( m_vImages, m_pMap, m_pTimer ) == false ) {
                std::cerr << "error: A problem was encountered initializing the front end." << std::endl;
                return false;
            }

            //----- load CVars
            std::string sCVars = clArgs.follow( "cvars.xml", "-cvars" );
            std::string sCVarsFile = sSrcDir + "/" + sCVars;
            if( CVarUtils::Load( sCVarsFile ) ) {
                std::cout << "Loading CVars file from: " << sCVarsFile << std::endl;
            }

            return true;
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void StepOnce( Gui& rGui )
        {
            m_pLocalizer->Tic();

            m_pLocalizer->Tic("Capture");
            bool bCapRet = m_Cam.Capture( m_vImages );
            m_pLocalizer->Toc("Capture");

            if( bCapRet ) {

                // unpack images to what the front end expects
                m_pLocalizer->Tic("Unpack");
                _UnpackImages( m_vImages );
                m_pLocalizer->Toc("Unpack");

                if( m_pLocalizer->Iterate( m_vImages ) == false ) {
                    std::cerr << "critical: something went wrong during the last iteration." << std::endl;
                }
                m_pLocalizer->Toc();

                // update analytics
                m_pLocalizer->GetAnalytics( m_Analytics );
                rGui.UpdateAnalytics( m_Analytics );
                rGui.UpdateTimer( m_pTimer->GetWindowSize(), m_pTimer->GetNames(3), m_pTimer->GetTimes(3) );

                // pause if we are lost
                if( m_pLocalizer->TrackingState() == eTrackingFail ) {
//                    rGui.SetState( RESETTING );
                }
            } else {
                // no more images, pause
                m_pLocalizer->Toc();
            }
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void UpdateGui( Gui& rGui )
        {
            rGui.CopyMapChanges( *m_pMap );
            rGui.UpdateImages( m_vImages[0].Image );
        }


    private:

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void _UnpackImages(
                CamImages&          vImages    //< Input/Output
            )
        {
            if( m_bConvertGrey ) {
                // convert RGB to GREYSCALE
                cv::cvtColor( vImages[0].Image, vImages[0].Image, CV_RGB2GRAY, 1 );
            }

            // TODO this ONLY works with fireyfly atm
            if( m_bRectify ) {
                cv::Mat Tmp;
                cv::Mat Intrinsics = (cv::Mat_<float>(3,3) <<    655.0681058933573, 0, 329.3888800064832,
                                                                  0, 651.5601207003715, 249.7271121691255,
                                                                  0, 0, 1);
                cv::Mat Distortion = (cv::Mat_<float>(1,5) <<   -0.4309355351200019, 0.2749971654145275, 0.002517876773074356, -0.0003738676467441764, -0.1696187437955576);

                cv::undistort( vImages[0].Image, Tmp, Intrinsics, Distortion );
                vImages[0].Image = Tmp;
            }

            if( m_bResize ) {
                cv::Mat Tmp;
                cv::resize( vImages[0].Image, Tmp, cv::Size(0,0), 0.5, 0.5 );
                vImages[0].Image = Tmp;
            }
        }


    /////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private:

        CameraDevice                    m_Cam;          // camera handler
        CamImages                       m_vImages;      // camera images

        /// auxilary variables to support multiple cameras, image pre-processing, etc.
        bool                            m_bRectify;     // rectify image
        bool                            m_bConvertGrey; // convert to greyscale
        bool                            m_bResize;      // reduce greyscale

        FastLocalizer*                  m_pLocalizer;
        DenseMap*                       m_pMap;

        Timer*                                                  m_pTimer;
        std::map< std::string,  std::pair< double, double > >   m_Analytics;

};

#endif
