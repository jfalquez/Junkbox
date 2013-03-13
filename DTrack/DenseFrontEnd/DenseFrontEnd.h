
#ifndef _DENSE_FRONT_END_H_
#define _DENSE_FRONT_END_H_

#include <Eigen/Dense>
#include <boost/thread.hpp>

#include <RPG/Utils/ImageWrapper.h>

#include <DenseMap/DenseMap.h>

#include "Timer.h"


typedef std::vector< rpg::ImageWrapper >               CamImages;

/*
 * The program expects the first image to be a rectified greyscale and the second
 * can be a depth map, but it is not required if a previous map is given. Otherwise
 * it is required for keyframe generation!!
 *
 * The initial map can be given, in which case only greyscale images are usually passed
 * or if no map is given it will generate keyframes. It is RESPONSABILITY of the
 * application calling this method to PACK correctly the images for the FrontEnd!!
 *
 * The PACKing must have:
 * - Image1 as GREYSCALE CV_8UC1 format.
 * - Image2 as DEPTH CV_32FC1 format in METERS.
 *
 */

enum eTrackingState
{
    eTrackingGood = 1,
    eTrackingPoor = 2,
    eTrackingBad  = 4,
    eTrackingFail = 8
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class DenseFrontEnd
{

public:

    DenseFrontEnd();

    ~DenseFrontEnd();

    // need to pass:
    // K for greyscale camera.
    // K for depth camera (in some cases these are the same).
    // Tid = Transform between intensity sensor (greyscale) and depth sensor.
    bool Init(
            const CamImages&        vImages,    //< Input: Camera Capture
            Eigen::Matrix3d         Ki,         //< Input: Intensity camera's intrinsics
            Eigen::Matrix3d         Kd,         //< Input: Depth camera's intrinsics
            Eigen::Matrix4d         Tid,        //< Input: Transform between intensity and depth camera
            DenseMap*               pMap,       //< Input: Pointer to the map that should be used
            Timer*                  pTimer      //< Input: Pointer to timer
        );

    bool Iterate(
            const CamImages&        vImages     //< Input: Camera Capture
        );

    Eigen::Matrix4d GetCurrentPose();

    bool TrackingBad()
    {
        return m_eTrackingState == eTrackingBad;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:

    void _SetCurTime( double dCurTime );

    // This function will localize a given frame against a reference frame
    bool _EstimateRelativePose( FramePtr pFrameA,
            FramePtr pFrameB,
            Eigen::Matrix4d& Tab
        );


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:

    unsigned long int                   m_nFrameIndex;              // frame index counter
    double                              m_dCurTime;
    eTrackingState                      m_eTrackingState;

    FramePtr                            m_pCurFrame;
    FramePtr                            m_pPrevFrame;

    Timer*                              m_pTimer;

    DenseMap*                           m_pMap;                     // map

    Eigen::Matrix4d                     m_dGlobalPose;              // global pose for display w.r.t first frame
    boost::mutex                        m_Mutex;
};


#endif