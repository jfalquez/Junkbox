
#ifndef _DENSE_FRONT_END_H_
#define _DENSE_FRONT_END_H_

#include <Eigen/Dense>
#include <boost/thread.hpp>

#include <RPG/Utils/ImageWrapper.h>

#include <DenseMap/DenseMap.h>
#include <Utils/CamModelPyramid.h>

#include "DenseFrontEndConfig.h"
#include "GpuHelpers.h"
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
    eTrackingGood           = 1,
    eTrackingPoor           = 2,
    eTrackingBad            = 4,
    eTrackingFail           = 8,
    eTrackingLoopClosure    = 16
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class DenseFrontEnd
{
public:

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    DenseFrontEnd( unsigned int nImageWidth, unsigned int nImageHeight );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ~DenseFrontEnd();

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// given a camera, initialize and reset the slam engine
    /// returns: true if no errors were encountered during initialization
    bool Init( const CamImages& vImages, DenseMap* pMap, Timer* pTimer );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// this is the main entry point that the enclosing application calls to advance the engine forward
    /// returns: true if no errors were encountered
    bool Iterate(
            const CamImages&        vImages     //< Input: Camera Capture
        );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// returns a map with info of the state of several varibles in the engine
    void GetAnalytics(
            std::map< std::string, std::pair< double, double > >&    mData
        );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// timer auxilary functions
    void Tic( const std::string& sName = "" )
    {
        if( m_pTimer ){
            m_pTimer->Tic( sName );
        }
    }

    void Toc( const std::string& sName = "" )
    {
        if( m_pTimer ){
            m_pTimer->Toc( sName );
        }
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// returns current tracking state
    eTrackingState TrackingState()
    {
        return m_eTrackingState;
    }

private:

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// this function will localize an image against a keyframe
    /// returns: root mean square error
    double _EstimateRelativePose(
            const cv::Mat&          GreyImg,        //< Input: Greyscale image
            FramePtr                pKeyframe,      //< Input: Keyframe we are localizing against
            Eigen::Matrix4d&        Tkc,            //< Input/Output: the estimated relative transform (input is used as a hint)
            unsigned int&           nNumObs         //< Output: Number of observations used for estimate
            );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// this function will try to relocalize the input image against the map
    /// returns: frame ID of keyframe used for relocalization -- returns -1 if no frame is found
    int _Relocalize(
            const cv::Mat&          Image,          //< Input: Image from which we want to relocalize
            Eigen::Matrix4d&        T,              //< Output: Transform between input frame and closest matching frame
            double&                 dError          //< Output: RMSE of estimated transform
        );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// this function will try to find a loop closure between the provided keyframe and the map
    /// returns: frame ID of loop closure -- returns -1 if no frame is found
    int _LoopClosure(
            FramePtr                pFrame,         //< Input: Frame we are attempting to find a loop closure
            Eigen::Matrix4d&        T,              //< Output: Transform between input frame and closest matching frame
            double&                 dError          //< Output: RMSE of estimated transform
        );

/////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
    eTrackingState                                          m_eTrackingState;

    FramePtr                                                m_pCurFrame;
    FramePtr                                                m_pPrevFrame;

    Eigen::Matrix4d                                         m_Tpc;              // estimate from previous to current frame

    DenseMap*                                               m_pMap;             // map use for estimating poses

    unsigned int                                            m_nImageWidth;
    unsigned int                                            m_nImageHeight;
    unsigned int                                            m_nThumbWidth;
    unsigned int                                            m_nThumbHeight;

    Timer*                                                  m_pTimer;
    std::map< std::string, std::pair< double, double > >    m_Analytics;        // statistics for display

    boost::mutex                                            m_Mutex;

    // GPU Variables
    Gpu::Pyramid< unsigned char, MAX_PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage >   m_cdGreyPyr;
    Gpu::Pyramid< unsigned char, MAX_PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage >   m_cdKeyGreyPyr;
    Gpu::Pyramid< float, MAX_PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage >           m_cdKeyDepthPyr;
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage >                     m_cdWorkspace;
    Gpu::Image< float4, Gpu::TargetDevice, Gpu::Manage >                            m_cdDebug;
    GpuVars_t                                                                       m_cdTemp;

};

#endif
