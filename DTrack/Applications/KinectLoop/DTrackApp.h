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

            //----- init camera 1 with default initializer
            m_Cam.SetProperty("BufferSize", 20);

            // initialize camera
            if( !rpg::InitCam( m_Cam, clArgs ) ) {
                exit(1);
            }

            // capture check
            m_vImages.clear();
            m_Cam.Capture( m_vImages );

            //----- get extra parameters of our own making
            m_bConvertGrey = clArgs.search( "-rgb-to-grey" );
            m_bRectify = clArgs.search( "-rectify" );
            m_bResize = clArgs.search( "-resize" );
            m_bConvertToM = clArgs.search( "-depth-to-m" );


            //----- init camera 2 if necessary
            std::string sDDev = clArgs.follow( "", "-ddev" );

            m_bAuxCam = false;
            if( sDDev.empty() == false ) {
                m_bAuxCam = true;
                std::cout << "External camera provided." << std::endl;

                bool            bGetDepth   = !clArgs.search( "-no-depth" );
                bool            bGetRGB     = !clArgs.search( "-no-rgb" );
                bool            bGetIr      = clArgs.search( "-with-ir" );
                bool            bAlignDepth = clArgs.search( "-align-depth" );
                unsigned int    nFPS        = clArgs.follow( 30, "-fps"  );
                std::string     sResolution = clArgs.follow( "VGA", "-res"  );

                m_CamAux.SetProperty( "GetRGB", bGetRGB );
                m_CamAux.SetProperty( "GetDepth", bGetDepth );
                m_CamAux.SetProperty( "GetIr", bGetIr );
                m_CamAux.SetProperty( "AlignDepth", bAlignDepth );
                m_CamAux.SetProperty( "FPS", nFPS );
                m_CamAux.SetProperty( "Resolution", sResolution );

                m_CamAux.InitDriver( sDDev );

                CamImages   vImages;      // camera images
                m_CamAux.Capture( vImages );
                m_vImages.resize( 2 );
                m_vImages[1] = vImages[0];
            }


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

            if (m_pMap->LoadCameraModels( sGreyCModFilename, sDepthCModFilename ) == false ) {
                std::cerr << "abort: There was a problem loading the camera model files!" << std::endl;
                exit(1);
            }

            if( m_pFrontEnd ) {
                delete m_pFrontEnd;
            }
            m_pFrontEnd = new DenseFrontEnd( m_vImages[0].width(), m_vImages[0].height() );
            return m_pFrontEnd->Init( m_vImages, m_pMap, m_pTimer );
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void StepOnce( Gui& rGui )
        {
            if( m_Cam.Capture( m_vImages ) ) {

                if( m_bAuxCam ) {
                    CamImages   vImages;      // camera images
                    m_CamAux.Capture( vImages );
                    m_vImages.resize( 2 );
                    m_vImages[1] = vImages[0];
                }

                m_pFrontEnd->Tic();

                // unpack images to what the front end expects
                m_pFrontEnd->Tic("Unpack");
                _UnpackImages( m_vImages );
                m_pFrontEnd->Toc("Unpack");

                if( m_pFrontEnd->Iterate( m_vImages ) == false) {
                    std::cerr << "critical: something went wrong during the last iteration." << std::endl;
                    rGui.SetState( PAUSED );
                }
                m_pFrontEnd->Toc();

                // update analytics
                m_pFrontEnd->GetAnalytics( m_Analytics );
                rGui.UpdateAnalytics( m_Analytics );
                rGui.UpdateTimer( m_pTimer->GetWindowSize(), m_pTimer->GetNames(3), m_pTimer->GetTimes(3) );

                // pause if certain conditions are met
                if( m_pFrontEnd->TrackingState() == eTrackingFail || m_pFrontEnd->TrackingState() == eTrackingLoopClosure ) {
//                    rGui.SetState( PAUSED );
                }
            } else {
                // no more images, pause
                rGui.SetState( PAUSED );
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

            if( m_bConvertToM ) {
                cv::Mat Tmp;
                vImages[1].Image.convertTo( Tmp, CV_32FC1 );
                Tmp = Tmp / 1000;
                vImages[1].Image = Tmp;
            }

        }


    /////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private:

        CameraDevice                    m_Cam;          // camera handler
        CamImages                       m_vImages;      // camera images

        /// auxilary variables to support multiple cameras, image pre-processing, etc.
        bool                            m_bAuxCam;      // if external (auxilary) camera is being used
        bool                            m_bRectify;     // rectify image
        bool                            m_bConvertGrey; // convert to greyscale
        bool                            m_bResize;     // reduce greyscale
        bool                            m_bConvertToM;  // convert to meters ... usually used for kinect
        CameraDevice                    m_CamAux;       // camera handler

        DenseFrontEnd*                  m_pFrontEnd;
        DenseMap*                       m_pMap;

        Timer*                                                  m_pTimer;
        std::map< std::string,  std::pair< double, double > >   m_Analytics;

};

#endif
