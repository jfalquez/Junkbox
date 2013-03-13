#ifndef _DTRACK_APP_
#define _DTRACK_APP_

//#include <boost/thread.hpp>
//#include <opencv2/opencv.hpp>

#include <RPG/Utils/InitCam.h>
#include <DenseFrontEnd/DenseFrontEnd.h>

#include "Gui.h"

class DTrackApp
{
    public:

        /////////////////////////////////////////////////////////////////////////////
        DTrackApp()
        {
            m_pFrontEnd = NULL;
            m_pMap = NULL;
        }

        /////////////////////////////////////////////////////////////////////////////
        bool InitResetCameras(
                int argc,                       //< Input:
                char** argv,                    //< Input:
                CameraDevice& camera,           //< Output:
                VehicleConfig& vconfig,         //< Output:
                mvl::StereoRectification& srect //< Output:
                )
        {
            // Initialize camera
            if( !rpg::InitCam(camera,argc,argv) ){
                return false;
            }

            // Initialize vehicle configuration
            std::string src_dir    = camera.GetProperty( "DataSourceDir", "." );
            std::string lcmod_file = camera.GetProperty( "CamModel-L", "" );
            std::string rcmod_file = camera.GetProperty( "CamModel-R", "" );

            if( lcmod_file.empty() || rcmod_file.empty() ) {
                std::cerr << "ERROR: Camera model not provided > Vehicle config not initialized!" << std::endl;
                return false;
            } else {
                // load vehicle config (k matrix, pose)
                mvl::CameraModel left_camera_model(  src_dir + "/" + lcmod_file );
                mvl::CameraModel right_camera_model( src_dir + "/" + rcmod_file );

                srect.Init(left_camera_model,right_camera_model);

                vconfig.m_dIntrinsics[0]  = srect.GetRectK(0);
                vconfig.m_dIntrinsics[1]  = srect.GetRectK(1);
                vconfig.m_vSensorPoses[0] = srect.GetRectPose(0);
                vconfig.m_vSensorPoses[1] = srect.GetRectPose(1);
                mvl::Print( vconfig.m_dIntrinsics[0], "Left Cam Model" );
                mvl::Print( vconfig.m_dIntrinsics[1], "Right Cam Model" );
                mvl::Print( vconfig.m_vSensorPoses[0], "Left Sensor Pose" );
                mvl::Print( vconfig.m_vSensorPoses[1], "Right Sensor Pose" );
            }
            return true;
        }


        /////////////////////////////////////////////////////////////////////////////
        bool InitReset( int argc, char** argv )
        {
            frames.clear();
            rframes.clear();

            // Initialize odometry engine with the first stero-pair
            frames.resize(2);
            rframes.resize(2);

            // Initialize system
            if( !InitResetCameras(argc,argv,camera,vconfig,srect) ){
                exit(0);
            }

            camera.Capture(frames);
            srect.Rectify( frames[0].Image,frames[1].Image,rframes[0].Image,rframes[1].Image );
            improc.DoStereoBrightnessCorrection( rframes[0].Image, rframes[1].Image );

            if( m_pTimer ){
                delete m_pTimer;
            }
            m_pTimer = new Timer;


            if( m_pMap ){
                delete m_pMap;
            }
            m_pMap = new Map;

            if( m_pFrontEnd ){
                delete m_pFrontEnd;
            }
            m_pFrontEnd = new DenseFrontEnd;
            m_pFrontEnd->Init( vconfig, rframes, m_pMap, m_pTimer );

            return true;
        }

        /////////////////////////////////////////////////////////////////////////////
        void StepOnce( Gui& rGui )
        {
            bool bRes = camera.Capture( frames );
            if( bRes ){
                m_pFrontEnd->Tic();
                srect.Rectify( frames[0].Image, frames[1].Image, rframes[0].Image, rframes[1].Image );
                improc.DoStereoBrightnessCorrection( rframes[0].Image, rframes[1].Image );
                m_pFrontEnd->Iterate( rframes );
                m_pFrontEnd->GetAnalytics( analytics );

                // update the gui
                rGui.SetVehicleConfig( vconfig );

                rGui.CopyMapChanges( *m_pMap );

                rGui.UpdateFrames( rframes[0].Image, rframes[1].Image );

                rGui.UpdateKeypoints(
                        m_pFrontEnd->GetCurrentKeypointsForDisplay(0),
                        m_pFrontEnd->GetCurrentKeypointsForDisplay(1) );

                rGui.UpdateActiveLandmarks( m_pFrontEnd->GetActiveLandmarks() );

                m_pFrontEnd->Toc();
                rGui.UpdateTimer( m_pTimer->GetWindowSize(), m_pTimer->GetNames(3), m_pTimer->GetTimes(3) );
                rGui.UpdateAnalytics( analytics );

                if( m_pFrontEnd->TrackingBad() ){
                    rGui.SetState( PAUSED );
                }
            }else{
                rGui.SetState( PAUSED );
            }
        }

        /////////////////////////////////////////////////////////////////////////////
        void UpdateGui( Gui& rGui )
        {
            rGui.SetVehicleConfig( vconfig );

            rGui.CopyMapChanges( *m_pMap );

            rGui.UpdateFrames( rframes[0].Image, rframes[1].Image );

            rGui.UpdateKeypoints(
                    m_pFrontEnd->GetCurrentKeypointsForDisplay(0),
                    m_pFrontEnd->GetCurrentKeypointsForDisplay(1)
                    );

            rGui.UpdateTimer( m_pTimer->GetWindowSize(),
                             m_pTimer->GetNames(3), m_pTimer->GetTimes(3) );

            rGui.UpdateAnalytics( analytics );

            rGui.UpdateActiveLandmarks( m_pFrontEnd->GetActiveLandmarks() );
        }

        /////////////////////////////////////////////////////////////////////////////
    private:
        DenseFrontEnd*             m_pFrontEnd;
        Map*                       m_pMap;

        mvl::StereoRectification   srect;
        CameraDevice               camera;
        VehicleConfig              vconfig;
};

#endif

