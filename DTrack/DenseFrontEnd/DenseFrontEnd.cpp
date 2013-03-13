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
        const CamImages&        vImages,    //< Input: Camera Capture
        Eigen::Matrix3d         Ki,         //< Input: Intensity camera's intrinsics
        Eigen::Matrix3d         Kd,         //< Input: Depth camera's intrinsics
        Eigen::Matrix4d         Tid,        //< Input: Transform between intensity and depth camera
        DenseMap*               pMap,       //< Input: Pointer to the map that should be used
        Timer*                  pTimer      //< Input: Pointer to timer
    )
{
    m_pMap = pMap;
    m_pTimer = pTimer;
    m_pTimer->SetWindowSize( 40 );

    // store intrinsics

    // if MAP exists, try to find closes keyframe via thumbnails

    // if MAP null or closes keyframe finder fails, create a new keyframe
//    m_pCurFrame = m_pMap->NewFrame( mvl::Tic() );
    m_dGlobalPose.setIdentity();


    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This is the main entry point that the enclosing application calls to advance
// the slam engine forward.
bool DenseFrontEnd::Iterate( const CamImages& vImages )
{
    mvl::PrintHandlerSetErrorLevel( feConfig.g_nErrorLevel );

    /*

    // Start timer for the main cycle
    Tic("FrontEnd");

    Eigen::Matrix4d Tab = Eigen::Matrix4d::Identity();

    m_nFrameIndex++;
    PrintMessage(1,"Frame: %d \n",m_nFrameIndex);

    // TODO: Get this from frames (time when they were captured)
    double dSensorTime = mvl::Tic();

    {
        // Now allocate a new frame in the map (not linked yet until localization succeeds).
        m_pPrevFrame = m_pCurFrame;
        m_pCurFrame  = m_pMap->NewFrame( dSensorTime );
        // swap feature ptrs;
        FeatureImageVector tmp = m_vpCurFeatureImages;
        m_vpCurFeatureImages = m_vpPrevFeatureImages;
        m_vpPrevFeatureImages = tmp;
    }

    // Do feature extraction
    Tic("ProcessImages");
    _ExtractFeatures( vImages );
    Toc("ProcessImages");

    // Use extracted features to localize the system against the old map.
    Tic("PoseEstimation");
    if( ! _EstimateRelativePose( m_pPrevFrame, m_pCurFrame, Tab ) ) {
        m_eTrackingState = eTrackingBad;
        //  The current function just copies the last transformation.
        // TODO: implement real relocalization with saved map
        if( !_Relocalization(Tab) ){
            m_eTrackingState = eTrackingFail;
        }
    }
    Toc("PoseEstimation");

    // Success, so link this new frame into the map
    m_pMap->LinkFrames( m_pPrevFrame, m_pCurFrame, Tab );

    if( feConfig.m_bDoBundleAdjustment ) {
        Tic("BA");
        if(m_nFrameIndex > 3) {
            //m_BackEndOptimizer.RefineMap(g_FrontEndConfig.m_uBAWindowSize);
            AlternatingBundleAdjustment(
                    m_pMap,
                    feConfig.m_uBAWindowSize,
                    feConfig.m_uBANumIter,
                    m_VehicleConfig,
                    feConfig.m_bUseInverseDepthParameterization
                   );
        }
        Toc("BA");
    }

    // Update quad tree, add new features if necessary.
    Tic("StartNewLandmarks");
    _StartNewLandmarks();
    Toc("StartNewLandmarks");

    Toc( "FrontEnd" );

    // check if we should add keyframe
    //    if(m_Keyframe.IsKeyframe(dTab,m_pCurFrame))
    //        m_Keyframe.SetKeyframe(vImages[0].Image,vImages[1].Image);

    */
    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4d DenseFrontEnd::GetCurrentPose()
{
    Eigen::Matrix4d Tab;
    m_pMap->GetTransformFromParent( m_pCurFrame->Id(), Tab );

    return Tab;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This function will localize a given frame against a reference frame.
bool DenseFrontEnd::_EstimateRelativePose(
        FramePtr pFrameA,
        FramePtr, // pFrameB,
        Eigen::Matrix4d& Tab  //< Output: the estimated transform
        )
{
    /*

    // Transform 3d position of landmarks with respect to the previous frame
    Tic("ComputeWorkingSet");
    m_WorkSet.ComputeWorkingSet( pFrameA->Id(), feConfig.m_uMatchInTimeWindowSize, m_VehicleConfig );
    m_dMeanTrackLength = m_WorkSet.GetMeanLandmarkTrackLength();
    Toc("ComputeWorkingSet");

    // Find matches of working set landmarks in the current frame
    // Cicle until enough matches have been found (60% of working set)
    Tic("MatchInTime");
    bool bEnoughMatches = false;
    int  nSearchFactor  = 1;
    std::vector<Measurement> vNewMeasurements;
    while(!bEnoughMatches) {
        // compute matches
        bEnoughMatches = _MatchInTime( Tab , nSearchFactor, vNewMeasurements );
        if( !bEnoughMatches ){
            // increase search area
            PrintMessage(0,"MIT FAILURE -> Increasing Search area \n");
            nSearchFactor++;
            if(nSearchFactor <= 5) {
                _ResetFeatures();
            }else{
                break;
            }
        }
    }
    Toc("MatchInTime");

    // RANSAC, flag inlier set, then do a GN optimization to find local pose.
    Tic("RANSAC");
    if( feConfig.m_bDoRANSAC ) {
        _RANSAC( Tab, vNewMeasurements );
    }

    int nNumInliers = _FlagRANSACOutliers( Tab, vNewMeasurements, m_dMeanReprojectionError);
    // failure case
    if( nNumInliers < (int)feConfig.m_uRANSACMinInliers ) {
        PrintMessage(0,"RANSAC FAILURE -> Using motion model \n");
        Toc("RANSAC");
        return false;
    }
    Toc("RANSAC");

    Tic("GaussNewton");
    if( feConfig.m_bUseCERES ) {
        _GaussNewtonCERES( Tab, vNewMeasurements );
    } else {
        _GaussNewton( Tab, vNewMeasurements );
    }
    nNumInliers = _FlagOutliers( Tab, vNewMeasurements, m_dMeanReprojectionError );
    m_mAnalytics["MRE"] = m_dMeanReprojectionError;
    Toc("GaussNewton");

    // add new measurements to current frame and to landmark tracks
    _AddNewMeasurementsToMap( vNewMeasurements );

    */
    return true;
}
