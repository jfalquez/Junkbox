#include <Mvlpp/Mvl.h>

#include "DenseFrontEnd.h"
#include "DenseFrontEndConfig.h"


// Global CVars
DenseFrontEndConfig         feConfig;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
DenseFrontEnd::DenseFrontEnd()
{
    mvl::PrintHandlerSetErrorLevel( feConfig.g_nErrorLevel );

    m_pMap   = NULL; // passed in by the user
    m_pTimer = NULL; // passed in by the user
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
DenseFrontEnd::~DenseFrontEnd()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Given a camera, initialize and reset the slam engine
bool DenseFrontEnd::Init(
        std::string             sGreyCModFilename,  //< Input:
        std::string             sDepthCModFilename, //< Input:
        const CamImages&        vImages,            //< Input: Camera capture
        DenseMap*               pMap,               //< Input: Pointer to the map that should be used
        Timer*                  pTimer              //< Input: Pointer to timer
    )
{
    m_pMap = pMap;

    m_pTimer = pTimer;
    m_pTimer->SetWindowSize( 40 );

    // get intrinsics
    if( !m_CModPyrGrey.Read( sGreyCModFilename ) ) {
        return false;
    }

    if( !m_CModPyrDepth.Read( sDepthCModFilename ) ) {
        return false;
    }

    // print intrinsics information
    std::cout << "Intensity Camera Intrinsics: " << std::endl;
    std::cout << m_CModPyrGrey.K() << std::endl << std::endl;
    std::cout << "Depth Camera Intrinsics: " << std::endl;
    std::cout << m_CModPyrDepth.K() << std::endl << std::endl;
    std::cout << "Tid: " << std::endl;
    std::cout << m_CModPyrDepth.GetPose() << std::endl << std::endl;

    // sanity check
    if( vImages[0].height() != m_CModPyrGrey.Height() ) {
        std::cerr << "warning: Camera model and captured image's height do not match. Are you using the correct CMod file?" << std::endl;
    }
    if( vImages[0].width() != m_CModPyrGrey.Width() ) {
        std::cerr << "warning: Camera model and captured image's width do not match. Are you using the correct CMod file?" << std::endl;
    }

    bool bNewFrame = true;

    // check if MAP is preloaded with frames...
    if( m_pMap->NumFrames() == 0 ) {

        // nope, so set path base pose and global pose accordingly
        m_pMap->SetPathBasePose( Eigen::Matrix4d::Identity() );
        m_dGlobalPose.setIdentity();

    } else {

        //... cool, there is a map. Let's see if we can find a keyframe we can use...
        // use thumbnails matching system to find closest keyframe
        // run ESM to get estimate
        // have some sort of condition that accepts or not the estimate (RMSE?)..
        // if ACCEPT: fix global pose accordingly
        // m_dBasePose = m_dGlobalPose = ESTIMATE;
        // m_pCurFrame == THAT keyframe chosen
            // IF THRESHOLD FOR NEW KEYFRAME IS *NOT* MET (therefore not needed), reset flag
                //bDropNewFrame = false;

        // if NOT ACCEPT, then drop new keyframe
        // bDropNewFrame = true; <---- this is already to true so do nothing

    }

    if( bNewFrame ) {
        _GenerateKeyframe( vImages );
        // TODO get this from the camera directly
        double dSensorTime = mvl::Tic();
        m_pCurKeyframe = m_pMap->NewFrame( dSensorTime, vImages[0].Image, vImages[1].Image );
        m_pMap->SetKeyframe( m_pCurKeyframe );
    }


    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// this is the main entry point that the enclosing application calls to advance the engine forward
bool DenseFrontEnd::Iterate(
        const CamImages&    vImages     //< Input: Camera capture
    )
{
    // update error level in case user changed it
    mvl::PrintHandlerSetErrorLevel( feConfig.g_nErrorLevel );

    // given a motion model or IMU, get estimated pose
    // based on this pose, load closest keyframe (euclidean distance)

    // run ESM to localize

    // drop estimate into path vector

    // update global pose

    // check point threshold to see if new keyframe must be added to the map

    return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DenseFrontEnd::_GenerateKeyframe(
        const CamImages&    vImages     //< Input: Images used to generate new keyframe
    )
{
    // check if two images (grey and depth) were provided
    if( vImages.size() < 2 ) {
        std::cerr << "warning: Could not create keyframe since two images are required!" << std::endl;
        return false;
    }

    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This function will localize a given frame against a reference frame.
bool DenseFrontEnd::_EstimateRelativePose(
        FramePtr pFrameA,
        FramePtr, // pFrameB,
        Eigen::Matrix4d& Tab  //< Output: the estimated transform
        )
{
}
