
#ifndef _DENSE_FRONT_END_H_
#define _DENSE_FRONT_END_H_

#include <map>
#include <vector>
#include <deque>

#include <DenseMap/DenseMap.h>

#include <RPG/Utils/ImageWrapper.h>

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
 */

//////////////////////////////////////////////////////////////////////////////////////////
class DenseFrontEnd
{

public:

    DenseFrontEnd();

    ~DenseFrontEnd();

    bool Init(
            const VehicleConfig&  Cfg,    //< Input: holds camera models etc.
            CamImages&            frames, //< Input: captured rectified stereo-pair
            Map*                  pMap,   //< Input: pointer to the map we should use
            );


    bool Iterate(CamFrames& frames);

    Eigen::Matrix4d GetCurrentPose();

    std::map< std::string, float > SystemStatus();


    //////////////////////////////////////////////////////////////
    /// Private member functions
private:

    void _Clear();

    void _SetCurTime( double dCurTime );

    // This function will localize a given frame against a reference frame
    bool _EstimateRelativePose( FramePtr pFrameA,
            FramePtr pFrameB,
            Eigen::Matrix4d& Tab );

    //////////////////////////////////////////////////////////////
    /// Private member data
private:

    unsigned long int                  m_nFrameIndex;              // frame index counter
    unsigned int                       m_nNumMITMatches;
    unsigned int                       m_nNumTrackedLandmarks;
    unsigned int                       m_nNumNewLandmarks;
    double                             m_dCurTime;
    double                             m_dTrackedFeat;
    double                             m_dMeanReprojectionError;
    double                             m_dEstDistanceTraveled;
    double                             m_dMeanTrackLength;
    double                             m_dInlierNoiseError;
    double                             m_dLearningRate;

    FramePtr                       m_pCurFrame;
    FramePtr                       m_pPrevFrame;

    VehicleConfig                      m_VehicleConfig;            // holds configuration info about the robot
    Map*                               m_pMap;                     // map


    Eigen::Matrix4d                    m_dGlobalPose;              // global pose for display w.r.t first frame
    std::vector<Eigen::Vector6d>       m_vGroundTruth;
    FeatureHandler                     m_vFeatureHandlers[NUM_CAMERAS]; // two because they run in separate threads!
    boost::threadpool::pool*           m_pThreadPool0;
    boost::threadpool::pool*           m_pThreadPool1;
    boost::mutex                       m_Mutex;
};


#endif
