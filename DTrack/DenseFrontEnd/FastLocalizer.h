#ifndef _FAST_LOCALIZER_H_
#define _FAST_LOCALIZER_H_

#include <Eigen/Dense>

#include <RPG/Utils/ImageWrapper.h>

#include <DenseMap/DenseMap.h>

#include "FastLocalizerConfig.h"
#include "GpuHelpers.h"
#include "Timer.h"


typedef std::vector< rpg::ImageWrapper >               CamImages;

enum eTrackingState
{
    eTrackingGood           = 1,
    eTrackingPoor           = 2,
    eTrackingBad            = 4,
    eTrackingFail           = 8
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class FastLocalizer
{
public:

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    FastLocalizer( unsigned int nImageWidth, unsigned int nImageHeight );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ~FastLocalizer();

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


/////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
    eTrackingState                                          m_eTrackingState;

    Eigen::Matrix4d                                         m_T_p_c;            // estimate from previous to current frame

    DenseMap*                                               m_pMap;             // map use for estimating poses

    unsigned int                                            m_nImageWidth;
    unsigned int                                            m_nImageHeight;
    unsigned int                                            m_nThumbWidth;
    unsigned int                                            m_nThumbHeight;

    Timer*                                                  m_pTimer;
    std::map< std::string, std::pair< double, double > >    m_Analytics;        // statistics for display

    // GPU Variables
    Gpu::Pyramid< unsigned char, MAX_PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage >   m_cdGreyPyr;
    Gpu::Pyramid< unsigned char, MAX_PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage >   m_cdKeyGreyPyr;
    Gpu::Pyramid< float, MAX_PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage >           m_cdKeyDepthPyr;
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage >                     m_cdWorkspace;
    Gpu::Image< float4, Gpu::TargetDevice, Gpu::Manage >                            m_cdDebug;
    GpuVars_t                                                                       m_cdTemp;

};

#endif
