#ifndef _DTRACK_APP_
#define _DTRACK_APP_

//#include <boost/thread.hpp>
//#include <opencv2/opencv.hpp>

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
        bool InitResetCamera(
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

            // Initialize vehicle configuration
            std::string src_dir             = m_Cam.GetProperty( "DataSourceDir", "." );
            std::string intensity_cmod_file = clArgs.follow( "cmod_i.xml", "-icmod" );
            std::string depth_cmod_file     = clArgs.follow( "cmod_d.xml", "-dcmod" );

            // load camera models (k matrix, pose)
            mvl::CameraModel intensity_camera_model;
            mvl::CameraModel depth_camera_model;

            if( !intensity_camera_model.Read( src_dir + "/" + intensity_cmod_file ) ) {
                std::cerr << "ERROR: Failed to open intensity camera file." << std::endl;
                exit(0);
            }
            if( !depth_camera_model.Read( src_dir + "/" + depth_cmod_file ) ) {
                std::cerr << "ERROR: Failed to open depth camera file." << std::endl;
                exit(0);
            }

            m_Ki = intensity_camera_model.K();
            m_Kd = depth_camera_model.K();
            m_Tid = depth_camera_model.GetPose();

            std::cout << "Intensity Cam Model: " << std::endl << m_Ki << std::endl << std::endl;
            std::cout << "Depth Cam Model: " << std::endl << m_Kd << std::endl << std::endl;
            std::cout << "Tid: " << std::endl << m_Tid << std::endl << std::endl;

            return true;
        }


        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool InitReset(
                int         argc,                       //< Input:
                char**      argv                        //< Input:
            )
        {
            // initialize system
            if( !InitResetCamera( argc, argv ) ) {
                exit(0);
            }

            m_vImages.clear();
            m_Cam.Capture( m_vImages );
            _UnpackImages( m_vImages );

            if( m_pTimer ) {
                delete m_pTimer;
            }
            m_pTimer = new Timer;

            if( m_pMap ) {
                delete m_pMap;
            }
            m_pMap = new DenseMap;

            if( m_pFrontEnd ) {
                delete m_pFrontEnd;
            }
            m_pFrontEnd = new DenseFrontEnd;
            m_pFrontEnd->Init( m_vImages, m_Ki, m_Kd, m_Tid, m_pMap, m_pTimer );

            return true;
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void StepOnce( Gui& rGui )
        {
            if( m_Cam.Capture( m_vImages ) ) {
                _UnpackImages( m_vImages );
//                m_pFrontEnd->Iterate( rframes );

                if( m_pFrontEnd->TrackingBad() ){
                    rGui.SetState( PAUSED );
                }
            } else {
                rGui.SetState( PAUSED );
            }
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void UpdateGui( Gui& rGui )
        {
//            rGui.CopyMapChanges( *m_pMap );

            rGui.UpdateImages( m_vImages[0].Image );

//            rGui.UpdateTimer( m_pTimer->GetWindowSize(), m_pTimer->GetNames(3), m_pTimer->GetTimes(3) );

        }


    private:

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void _UnpackImages(
                CamImages&          vImages    //< Input/Output
            )
        {
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
        }


    private:

        CameraDevice                m_Cam;          // camera handler
        CamImages                   m_vImages;      // camera images
        Eigen::Matrix3d             m_Ki;           // intensity (ie. greyscale) camera's intrinsics
        Eigen::Matrix3d             m_Kd;           // depth camera's intrinsics
        Eigen::Matrix4d             m_Tid;          // depth camera's pose w.r.t. the greyscale camera

        DenseFrontEnd*              m_pFrontEnd;
        DenseMap*                   m_pMap;
        Timer*                      m_pTimer;
};

#endif
