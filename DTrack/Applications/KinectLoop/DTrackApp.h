#ifndef _DTRACK_APP_
#define _DTRACK_APP_

#include <RPG/Utils/InitCam.h>
#include <Mvlpp/Mvl.h>
#include <DenseFrontEnd/DenseFrontEnd.h>

#include "Gui.h"

//#define CAMAUX

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
                exit(1);
            }

            // capture check
            m_vImages.clear();
            m_Cam.Capture( m_vImages );

#ifdef CAMAUX
            m_CamAux.SetProperty( "GetRGB", false );
            m_CamAux.SetProperty( "GetDepth", true );
            m_CamAux.SetProperty( "GetIr", false );
            m_CamAux.SetProperty( "Resolution", "QVGA" );
            m_CamAux.InitDriver( "Kinect" );

            CamImages   vImages;      // camera images

            vImages.clear();
            m_CamAux.Capture( vImages );
            m_vImages.resize( 2 );

            m_vImages[1] = vImages[0];
#endif

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

#ifdef CAMAUX
            CamImages   vImages;      // camera images

            m_CamAux.Capture( vImages );
            m_vImages.resize( 2 );

            m_vImages[1] = vImages[0];
#endif

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

#ifdef CAMAUX
            /// for LIVE handheld

            /*
            Eigen::Matrix4d Ext;
            Ext(0,3) =  -0.01912135240773571;
            Ext(1,3) = -0.04023697389323921;
            Ext(2,3) = 0.03031353438241784;

            Ext.block<3,3>(0,0) << 0.9965421258143921, -0.005812579181767699, 0.08288549571902262,
                                    0.002862221277912812, 0.9993595214306352, 0.03567008011328222,
                                    -0.08303974450038187, -0.03530950083458261, 0.9959204988271524;

            std::cout << "Extrinsics: " << mvl::T2Cart(Ext).transpose() << std::endl;
            /* */

            // rgb image
            cv::Mat Tmp1, Tmp2;

            cv::Mat Intrinsics = (cv::Mat_<float>(3,3) <<    655.0681058933573, 0, 329.3888800064832,
                                                              0, 651.5601207003715, 249.7271121691255,
                                                              0, 0, 1);
            cv::Mat Distortion = (cv::Mat_<float>(1,5) <<   -0.4309355351200019, 0.2749971654145275, 0.002517876773074356, -0.0003738676467441764, -0.1696187437955576);

            cv::undistort( vImages[0].Image, Tmp1, Intrinsics, Distortion );
            cv::resize( Tmp1, vImages[0].Image, cv::Size(0,0), 0.5, 0.5 );
//            vImages[0].Image = Tmp1;

            // depth image
            vImages[1].Image.convertTo( Tmp2, CV_32FC1 );
            Tmp2 = Tmp2 / 1000;
            vImages[1].Image = Tmp2;
#endif

            /// for JPL data
            /*
            cv::Mat Tmp;

            cv::resize( vImages[0].Image, Tmp, cv::Size(0,0), 0.5, 0.5 );

            vImages[0].Image = Tmp;
            /* */

            /// for KINECT data
            /*
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

        CameraDevice                    m_Cam;          // camera handler
        CamImages                       m_vImages;      // camera images

#ifdef CAMAUX
        CameraDevice                    m_CamAux;       // camera handler
#endif

        DenseFrontEnd*                  m_pFrontEnd;
        DenseMap*                       m_pMap;

        Timer*                                                  m_pTimer;
        std::map< std::string,  std::pair< double, double > >   m_Analytics;

};

#endif
