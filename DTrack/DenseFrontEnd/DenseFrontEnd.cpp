#include <Mvlpp/Mvl.h>

#include "DenseFrontEnd.h"


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
    std::cout << mvl::T2Cart(m_CModPyrDepth.GetPose()).transpose() << std::endl << std::endl;

    // store image dimensions
    m_nImageWidth = vImages[0].width();
    m_nImageHeight = vImages[0].height();
    m_nThumbHeight = m_nImageHeight >> MAX_PYR_LEVELS-1;
    m_nThumbWidth = m_nImageWidth >> MAX_PYR_LEVELS-1;

    // sanity check
    if( m_nImageHeight != m_CModPyrGrey.Height() ) {
        std::cerr << "warning: Camera model and captured image's height do not match. Are you using the correct CMod file?" << std::endl;
    }
    if( m_nImageWidth != m_CModPyrGrey.Width() ) {
        std::cerr << "warning: Camera model and captured image's width do not match. Are you using the correct CMod file?" << std::endl;
    }

    // initialize CUDA variables
    if( CheckMemoryCUDA() < 384 ) {
        std::cerr << "error: There seems to be too little CUDA memory available! Aborting." << std::endl;
        return false;
    }
    m_cdWorkspace = Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage >( m_nImageWidth * sizeof(Gpu::LeastSquaresSystem<float,6>), m_nImageHeight );
    m_cdDebug = Gpu::Image< float4, Gpu::TargetDevice, Gpu::Manage >( m_nImageWidth, m_nImageHeight );
    m_cdGreyPyr.Allocate( m_nImageWidth, m_nImageHeight );
    m_cdKeyGreyPyr.Allocate( m_nImageWidth, m_nImageHeight );
    m_cdKeyDepthPyr.Allocate( m_nImageWidth, m_nImageHeight );
    m_cdTemp.Init( m_nImageWidth, m_nImageHeight );


    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //

    // assign map
    m_pMap = pMap;

    // assign timer
    m_pTimer = pTimer;
    m_pTimer->SetWindowSize( 40 );

    // flag used to generate a new keyframe
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
        m_pCurKeyframe = _GenerateKeyframe( vImages );
        if( m_pCurKeyframe == NULL ) {
            return false;
        }
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
FramePtr DenseFrontEnd::_GenerateKeyframe(
        const CamImages&    vImages     //< Input: Images used to generate new keyframe
    )
{
    // check if two images (grey and depth) were provided
    if( vImages.size() < 2 ) {
        std::cerr << "error: Could not create keyframe since two images are required!" << std::endl;
        return FramePtr( (ReferenceFrame*)NULL );
    }

    // allocate thumb images
    cv::Mat GreyThumb( m_nThumbHeight, m_nThumbWidth, CV_8UC1 );
    cv::Mat DepthThumb( m_nThumbHeight, m_nThumbWidth, CV_32FC1 );

    GenerateGreyThumbnail( m_cdTemp, vImages[0].Image, GreyThumb );
    GenerateDepthThumbnail( m_cdTemp, vImages[1].Image, DepthThumb );

    // TODO get this from the camera directly.. the property map should have it
    double dSensorTime = mvl::Tic();

    return m_pMap->NewFrame( dSensorTime, vImages[0].Image, vImages[1].Image, GreyThumb, DepthThumb );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DenseFrontEnd::_EstimateRelativePose(
        const cv::Mat&          GreyImg,        //< Input: Greyscale image
        FramePtr                pKeyframe,      //< Input: Keyframe we are localizing against
        Eigen::Matrix4d&        Tab             //< Output: the estimated transform
        )
{
}
