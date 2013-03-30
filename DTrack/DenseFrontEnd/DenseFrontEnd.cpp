#include <Mvlpp/Mvl.h>

#include <pangolin/pangolin.h>
#include <cuda_gl_interop.h>

#include "DenseFrontEnd.h"

#include "ImageHelpers.h"


// Global CVars
DenseFrontEndConfig         feConfig;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
DenseFrontEnd::DenseFrontEnd( unsigned int nImageWidth, unsigned int nImageHeight ) :
    m_cdGreyPyr( nImageWidth, nImageHeight ),
    m_cdKeyGreyPyr( nImageWidth, nImageHeight ),
    m_cdKeyDepthPyr( nImageWidth, nImageHeight ),
    m_cdWorkspace( nImageWidth * sizeof(Gpu::LeastSquaresSystem<float,6>), nImageHeight ),
    m_cdDebug( nImageWidth, nImageHeight ),
    m_cdTemp( nImageWidth, nImageHeight )
{
    mvl::PrintHandlerSetErrorLevel( feConfig.g_nErrorLevel );

    m_nImageWidth   = nImageWidth;
    m_nImageHeight  = nImageHeight;

    // calculate thumb image dimensions
    m_nThumbHeight = m_nImageHeight >> MAX_PYR_LEVELS-1;
    m_nThumbWidth = m_nImageWidth >> MAX_PYR_LEVELS-1;

    // initialize max number of iterations to perform at each pyramid level
    // level 0 is finest (ie. biggest image)
    feConfig.g_vPyrMaxIters.resize( MAX_PYR_LEVELS );
    feConfig.g_vPyrMaxIters.setZero();
    feConfig.g_vPyrMaxIters << 1, 2, 3, 4, 5;

    // initialize if full estimate should be performed at a particular level
    // 1: full estimate          0: just rotation
    feConfig.g_vPyrFullMask.resize( MAX_PYR_LEVELS );
    feConfig.g_vPyrFullMask.setZero();
    feConfig.g_vPyrFullMask << 1, 1, 1, 1, 0;

    m_pMap   = NULL; // passed in by the user
    m_pTimer = NULL; // passed in by the user

    // check CUDA
    CheckMemoryCUDA();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
DenseFrontEnd::~DenseFrontEnd()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DenseFrontEnd::Init(
        const CamImages&        vImages,            //< Input: Camera capture
        DenseMap*               pMap,               //< Input: Pointer to the map that should be used
        Timer*                  pTimer              //< Input: Pointer to timer
    )
{
    // assign map
    m_pMap = pMap;

    // assign timer
    m_pTimer = pTimer;
    m_pTimer->SetWindowSize( 40 );

    // print intrinsics information
    std::cout << "Greyscale Camera Intrinsics: " << std::endl;
    std::cout << m_pMap->GetGreyCameraK() << std::endl << std::endl;
    std::cout << "Depth Camera Intrinsics: " << std::endl;
    std::cout << m_pMap->GetDepthCameraK() << std::endl << std::endl;
    std::cout << "Tgd: " << std::endl;
    std::cout << mvl::T2Cart(m_pMap->GetDepthCameraPose()).transpose() << std::endl << std::endl;


    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //


    // flag used to generate a new keyframe
    bool bNewKeyframe = true;

    // check if MAP is preloaded with frames...
    if( m_pMap->GetNumFrames() == 0 ) {

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

    if( bNewKeyframe ) {

        // TODO get this from the camera directly.. the property map should have it
        double dSensorTime = mvl::Tic();

        // allocate thumb images
        cv::Mat GreyThumb( m_nThumbHeight, m_nThumbWidth, CV_8UC1 );
        cv::Mat DepthThumb( m_nThumbHeight, m_nThumbWidth, CV_32FC1 );

        // generate thumbnails
        GenerateGreyThumbnail( m_cdTemp, vImages[0].Image, GreyThumb );
        GenerateDepthThumbnail( m_cdTemp, vImages[1].Image, DepthThumb );

        m_pCurFrame = m_pMap->NewKeyframe( dSensorTime, vImages[0].Image, vImages[1].Image, GreyThumb, DepthThumb );

        if( m_pCurFrame == NULL ) {
            return false;
        }
        m_pMap->SetKeyframe( m_pCurFrame );
    }

    // reset Tpc
    m_Tpc.setIdentity();

    // keep internal map's path up to date
    m_pMap->UpdateInternalPath();

    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DenseFrontEnd::Iterate(
        const CamImages&    vImages     //< Input: Camera capture
    )
{
    // update error level in case user changed it
    mvl::PrintHandlerSetErrorLevel( feConfig.g_nErrorLevel );

    /*
     *      The map's global poses are updated at the end of every iteration with respect to the last frame.
     *      So in essence, the previous frame is the new origin in global coordinates: Twp = I4
     */
    Eigen::Matrix4d Twc; Twc.setIdentity();
    Eigen::Matrix4d Twp; Twp.setIdentity();

    // given a motion model or IMU, get estimated pose
    if( feConfig.g_bConstantVelocityMotionModel == true ) {
        Twc = m_Tpc;
    }

    // based on this pose, load closest keyframe (euclidean distance including rotation of sorts)
    std::vector< std::pair< unsigned int, float > > vNearKeyframes;

    if( feConfig.g_bAlwaysUseLastKeyframe == false ) {
        m_pMap->FindClosestKeyframes( Twc, feConfig.g_fCloseKeyframeNorm, vNearKeyframes );
    }

    // TODO: if this returns empty, run fast relocalizer (ie. all thumbnails).. if fast reloc fails, then break the map
    // and run a slower relocalizer in the background

    // set keyframe for localization
    if( vNearKeyframes.empty() == false ) {
        // if not empty, use closest keyframe
        // TODO: refine closest keyframes with thumbnail matching to really narrow it down??
        if( m_pMap->SetKeyframe( std::get<0>(vNearKeyframes[0]) ) == false ) {
            PrintMessage( 1, "warning: there was an error setting closest keyframe.\n" );
        }
    }

    unsigned int nKeyframeId = m_pMap->GetCurrentKeyframe()->GetId();
    std::map< unsigned int, Eigen::Matrix4d >& vPath = m_pMap->GetInternalPath();
    Eigen::Matrix4d Twk = vPath[nKeyframeId];

    // Tkc is what the estimator will gives us back
    // we can seed this via: Tkc = Tkw * Twc = TInv( Twk ) * Twc
    Eigen::Matrix4d Tkc = mvl::TInv( Twk ) * Twc;

    Tic("PoseEstimate");
    unsigned int nNumObservations;
    double dTrackingError = _EstimateRelativePose( vImages[0].Image, m_pMap->GetCurrentKeyframe(), Tkc, nNumObservations );
    m_Analytics["RMSE"] = std::pair<double, double>( dTrackingError, 0 );

    // we would also like to get Tpc: transform from previous frame to current frame
    // we obtain this via: Tpc = Tpk * Tkc
    // given: Tpk = Tpw * Twk = TInv( Twp ) * Twk
    Eigen::Matrix4d Tpc = mvl::TInv( Twp ) * Twk * Tkc;
    m_Tpc = Tpc;
    {
        Eigen::Vector6d E = mvl::T2Cart(Tpc);
        PrintMessage( 1, "--- Estimate: [ %f, %f, %f, %f, %f, %f ]\n", E(0), E(1), E(2), E(3), E(4), E(5) );
    }
    Toc("PoseEstimate");


    if( dTrackingError < 5 ) {
        m_eTrackingState = eTrackingGood;
    } else if( dTrackingError < 20 ) {
        m_eTrackingState = eTrackingPoor;
    } else if( dTrackingError < 35 ) {
        PrintMessage( 1, "warning: tracking is bad. (RMSE: %f)\n", dTrackingError );
        m_eTrackingState = eTrackingBad;
    } else {
        PrintMessage( 1, "warning: tracking failed. (RMSE: %f)\n", dTrackingError );
        m_eTrackingState = eTrackingFail;
        // TODO if here, run some sort of relocalizer?
    }

    // TODO get this from the camera directly.. the property map should have it
    double dSensorTime = mvl::Tic();

    // drop new keyframe if number of observations is too low
    double dObsThreshold = feConfig.g_fKeyframePtsThreshold * m_nImageHeight * m_nImageWidth;
    m_Analytics["Num Obs"] = std::pair<double, double>( nNumObservations, dObsThreshold );
    if( nNumObservations < dObsThreshold ) {

        Tic("GenKeyframe");

        // allocate thumb images
        cv::Mat GreyThumb( m_nThumbHeight, m_nThumbWidth, CV_8UC1 );
        cv::Mat DepthThumb( m_nThumbHeight, m_nThumbWidth, CV_32FC1 );

        // generate thumbnails
        GenerateGreyThumbnail( m_cdTemp, vImages[0].Image, GreyThumb );
        GenerateDepthThumbnail( m_cdTemp, vImages[1].Image, DepthThumb );

        // update frame pointers
        m_pPrevFrame = m_pCurFrame;

        m_pCurFrame = m_pMap->NewKeyframe( dSensorTime, vImages[0].Image, vImages[1].Image, GreyThumb, DepthThumb );

        if( m_pCurFrame == NULL ) {
            std::cerr << "error: generating new keyframe." << std::endl;
            Toc("GenKeyframe");
            return false;
        }

        // link previous frame with new frame (relative pose)
        m_pMap->LinkFrames( m_pPrevFrame, m_pCurFrame, Tpc );

        // set new keyframe as current keyframe
        m_pMap->SetKeyframe( m_pCurFrame );

        m_Analytics["Keyframes"] = std::pair<double, double>( 1.0, 0 );

        Toc("GenKeyframe");
    } else {        // otherwise drop a regular frame

        // allocate thumb image
        cv::Mat GreyThumb( m_nThumbHeight, m_nThumbWidth, CV_8UC1 );

        // generate thumbnail
        GenerateGreyThumbnail( m_cdTemp, vImages[0].Image, GreyThumb );

        // update frame pointers
        m_pPrevFrame = m_pCurFrame;

        m_pCurFrame = m_pMap->NewFrame( dSensorTime, vImages[0].Image, GreyThumb );

        if( m_pCurFrame == NULL ) {
            std::cerr << "error: generating new frame." << std::endl;
            return false;
        }

        // link previous frame with new frame (relative pose)
        m_pMap->LinkFrames( m_pPrevFrame, m_pCurFrame, Tpc );

        m_Analytics["Keyframes"] = std::pair<double, double>( 0, 0 );
    }

    // update internal path based on this new frame added
    m_pMap->UpdateInternalPath();

    // check for loop closure
    Tic("LoopClosure");
    double          dLoopClosureError;
    Eigen::Matrix4d LoopClosureT;
    int nLoopClosureFrameId = _LoopClosure( m_pCurFrame, LoopClosureT, dLoopClosureError );
    m_Analytics["LC RMSE"] = std::pair<double, double>( dLoopClosureError, feConfig.g_dLoopClosureThreshold );
    if( nLoopClosureFrameId != -1 && dLoopClosureError < feConfig.g_dLoopClosureThreshold ) {
        PrintMessage( 1, "Loop Closure Detected!!! Matching Frame: %d (RMSE: %f)\n", nLoopClosureFrameId, dLoopClosureError );
        m_eTrackingState = eTrackingLoopClosure;

        // link frames
        m_pMap->LinkFrames( m_pMap->GetFramePtr(nLoopClosureFrameId), m_pCurFrame, LoopClosureT );
//        m_pMap->LinkFrames( m_pCurFrame, m_pMap->GetFramePtr(nLoopClosureFrameId),  LoopClosureT.inverse() );

        // update internal path based on the loop closure
        m_pMap->UpdateInternalPath();
    }
    Toc("LoopClosure");

    return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DenseFrontEnd::GetAnalytics(
        std::map< std::string, std::pair< double, double > >&    mData
    )
{
    mData = m_Analytics;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double DenseFrontEnd::_EstimateRelativePose(
        const cv::Mat&          GreyImg,        //< Input: Greyscale image
        FramePtr                pKeyframe,      //< Input: Keyframe we are localizing against
        Eigen::Matrix4d&        Tkc,            //< Input/Output: the estimated relative transform (input is used as a hint)
        unsigned int&           nNumObs         //< Output: Number of observations used for estimate
        )
{
    // get GUI variables
    pangolin::Var<bool>             ui_bBreakEarly( "ui.Break Early" );
    pangolin::Var<float>            ui_fBreakErrorThreshold( "ui.Break Early Error Threshold" );
    pangolin::Var<unsigned int>     ui_nBlur("ui.Blur");
    pangolin::Var<float>            ui_fNormC( "ui.Norm C" );
    pangolin::Var<bool>             ui_bDiscardMaxMin( "ui.Discard Max-Min Pix Values" );

    //--- normalize GREYSCALE images
    cv::Mat lGreyImg = GreyImg.clone();
    BrightnessCorrectionImagePair( lGreyImg.data, pKeyframe->GetGreyImagePtr(), m_nImageHeight*m_nImageWidth );


    Tic("Upload");
    //--- upload GREYSCALE data to GPU as a pyramid
    m_cdGreyPyr[0].MemcpyFromHost( lGreyImg.data, m_nImageWidth );
    m_cdKeyGreyPyr[0].MemcpyFromHost( pKeyframe->GetGreyImagePtr() );

    for( int ii = 0; ii < ui_nBlur; ii++ ) {
        Gpu::Blur( m_cdGreyPyr[0], m_cdTemp.uImg1 );
        Gpu::Blur( m_cdKeyGreyPyr[0], m_cdTemp.uImg2 );
    }

    // downsample grey images
    Gpu::BlurReduce<unsigned char, MAX_PYR_LEVELS, unsigned int>( m_cdGreyPyr, m_cdTemp.uImg1, m_cdTemp.uImg2 );
    Gpu::BlurReduce<unsigned char, MAX_PYR_LEVELS, unsigned int>( m_cdKeyGreyPyr, m_cdTemp.uImg1, m_cdTemp.uImg2 );


    //--- upload DEPTH data to GPU as a pyramid
    m_cdKeyDepthPyr[0].MemcpyFromHost( pKeyframe->GetDepthImagePtr() );

    // downsample depth maps
    Gpu::BoxReduce<float, MAX_PYR_LEVELS, float>( m_cdKeyDepthPyr );
    Toc("Upload");

    //--- localize
    // IMPORTANT!!!!!!!
    // The localization code operates in VISION frame. Thus, we convert our initial pose from ROBOTICS to VISION
    // and once the solution is found, we convert it back from VISION to ROBOTICS.

    // vision-robotics permutation matrix
    Eigen::Matrix4d Tvr;
    Tvr << 0, 1, 0, 0,
           0, 0, 1, 0,
           1, 0, 0, 0,
           0, 0, 0, 1;

    // convert initial pose to VISION reference frame
    Tkc = Tvr * Tkc * Tvr.inverse();

    double          dLastError;

    Tic("Localize");
    for( int PyrLvl = MAX_PYR_LEVELS-1; PyrLvl >= 0; PyrLvl-- ) {
        dLastError = DBL_MAX;
        for(int ii = 0; ii < feConfig.g_vPyrMaxIters[PyrLvl]; ii++ ) {
            const unsigned              PyrLvlWidth = m_nImageWidth >> PyrLvl;
            const unsigned              PyrLvlHeight = m_nImageHeight >> PyrLvl;

            Eigen::Matrix3d             Kg = m_pMap->GetGreyCameraK( PyrLvl );  // grey sensor's instrinsics
            Eigen::Matrix3d             Kd = m_pMap->GetGreyCameraK( PyrLvl );  // depth sensor's intrinsics
            Eigen::Matrix4d             Tgd = m_pMap->GetDepthCameraPose();     // depth sensor's pose w.r.t. grey sensor
            Eigen::Matrix4d             Tck = Tkc.inverse();
            Eigen::Matrix<double,3,4>   KgTck = Kg * Tck.block<3,4>(0,0);       // precompute for speed

            const float fNormC = ui_fNormC * ( 1 << PyrLvl );

            // build system
            Gpu::LeastSquaresSystem<float,6> LSS = Gpu::PoseRefinementFromDepthESM( m_cdGreyPyr[PyrLvl],
                                                                                    m_cdKeyGreyPyr[PyrLvl],
                                                                                    m_cdKeyDepthPyr[PyrLvl],
                                                                                    Kg, Kd, Tgd, Tck, KgTck,
                                                                                    m_cdWorkspace, m_cdDebug.SubImage(PyrLvlWidth, PyrLvlHeight),
                                                                                    fNormC, ui_bDiscardMaxMin, 0.3, 30.0 );

// Show DEBUG image from minimization
#if 0
            {
                cv::Mat Debug( m_nImageHeight, m_nImageWidth, CV_32FC4 );
                m_cdDebug.MemcpyToHost(Debug.data);
                cv::imshow("Debug", Debug );
                cv::waitKey(1000);
                if( PyrLvl == 0 ) {
                    cv::waitKey(3000);
                    cv::destroyWindow("Debug");
                }
            }
#endif

            Eigen::Matrix<double,6,6>   LHS = LSS.JTJ;
            Eigen::Vector6d             RHS = LSS.JTy;

            // solve system
            Eigen::Vector6d             X;

            // check if we are solving only for rotation, or full estimate
            if( feConfig.g_vPyrFullMask(PyrLvl) != 0 ) {
                Eigen::FullPivLU< Eigen::Matrix<double,6,6> >    lu_JTJ(LHS);

                // check degenerate system
                if( lu_JTJ.rank() < 6 ) {
                    PrintMessage( feConfig.g_nDebugESM, "warning(@L%d I%d) LS trashed. Rank deficient!\n", PyrLvl+1, ii+1 );
                    continue;
                }

                X = - (lu_JTJ.solve(RHS));
            } else {
                // extract rotation information only
                Eigen::Matrix<double,3,3>                       rLHS = LHS.block<3,3>(3,3);
                Eigen::Vector3d                                 rRHS = RHS.tail(3);
                Eigen::FullPivLU< Eigen::Matrix<double,3,3> >   lu_JTJ(rLHS);

                // check degenerate system
                if( lu_JTJ.rank() < 3 ) {
                    PrintMessage( feConfig.g_nDebugESM, "warning(@L%d I%d) LS trashed. Rank deficient!\n", PyrLvl+1, ii+1 );
                    continue;
                }

                Eigen::Vector3d rX;
                rX = - (lu_JTJ.solve(rRHS));

                // pack solution
                X.setZero();
                X.tail(3) = rX;
            }

            /*
            // if we have too few observations, discard estimate
            if( (float)LSS.obs < ui_fMinPts * (float)( PyrLvlWidth * PyrLvlHeight ) ) {
                std::cerr << "warning(@L" << PyrLvl+1 << "I" << ii+1 << ") LS trashed. " << "Too few pixels!" << std::endl;
                continue;
            }
            /* */

            // everything seems fine... apply update
            Tkc = (Tkc.inverse() * Sophus::SE3::exp(X).matrix()).inverse();

            const double dNewError = sqrt(LSS.sqErr / LSS.obs);

            // if error decreases too slowly, break out of this level
            if( ( fabs( dNewError - dLastError ) < ui_fBreakErrorThreshold ) && ui_bBreakEarly ) {
                PrintMessage( feConfig.g_nDebugESM, "notice: Breaking early @ L%d : I%d ...\n", PyrLvl+1, ii+1 );
                break;
            }

            // update error & number of observations
            dLastError = dNewError;
            nNumObs = LSS.obs;
        }
    }
    Toc("Localize");

    // convert estimate to ROBOTICS frame
    Tkc = Tvr.inverse() * Tkc * Tvr;

    return dLastError;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int DenseFrontEnd::_LoopClosure(
        FramePtr                pFrame,         //< Input: Frame we are attempting to find a loop closure
        Eigen::Matrix4d&        T,              //< Output: Transform between input frame and closest matching frame
        double&                 dError          //< Output: RMSE of estimated transform
    )
{
    const unsigned int nFrameId = pFrame->GetId();

    std::map< unsigned int, Eigen::Matrix4d >& vPath = m_pMap->GetInternalPath();
    Eigen::Matrix4d Pose = vPath[nFrameId];

    // find closest frames around our current position
    std::vector< std::pair< unsigned int, float > > vNearKeyframes;
    m_pMap->FindClosestKeyframes( Pose, feConfig.g_fLoopClosureRadius, vNearKeyframes );
    if( vNearKeyframes.empty() ) {
        return -1;
    }

    // TODO add grey-depth contribution ratio for matching

    float       fBestScore = FLT_MAX;
    FramePtr    pBestMatch;

    for( int ii = 0; ii < vNearKeyframes.size(); ++ii ) {

        const unsigned int nId = std::get<0>( vNearKeyframes[ii] );

        // do not try to compare with frames too close too us
        if( abs(nId - nFrameId) <= feConfig.g_nLoopClosureMargin ) {
            continue;
        }

        // get frame pointer
        FramePtr pMatchFrame = m_pMap->GetFramePtr( nId );

        // do not compare with non-keyframes
        if( pMatchFrame->IsKeyframe() == false ) {
            continue;
        }

        float fScore = ScoreImages<unsigned char>( pFrame->GetGreyThumbRef(), pMatchFrame->GetGreyThumbRef() );

        if( fScore < fBestScore ) {
            fBestScore = fScore;
            pBestMatch = pMatchFrame;
        }
    }

    double dSADThreshold = feConfig.g_nLoopClosureSAD * (m_nThumbHeight * m_nThumbWidth);

    m_Analytics["LC SAD"] = std::pair<double, double>( fBestScore, dSADThreshold );

    // if no match is found, return -1
    if( fBestScore >  dSADThreshold ) {
        return -1;
    }

    PrintMessage( 1, "Loop Closure candidate found! (SAD: %f)\n", fBestScore );

    // calculate transform between best match and input frame
    unsigned int nNumObservations;
    dError = _EstimateRelativePose( pFrame->GetGreyImageRef(), pBestMatch, T, nNumObservations );

    return pBestMatch->GetId();
}

