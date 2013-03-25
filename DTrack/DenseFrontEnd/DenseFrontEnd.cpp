#include <Mvlpp/Mvl.h>

#include <pangolin/pangolin.h>
#include <cuda_gl_interop.h>

#include "DenseFrontEnd.h"


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

    m_nImageWidth = nImageWidth;
    m_nImageHeight = nImageHeight;

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
    std::cout << "Tgd: " << std::endl;
    std::cout << mvl::T2Cart(m_CModPyrDepth.GetPose()).transpose() << std::endl << std::endl;

    // sanity check
    if( m_nImageHeight != m_CModPyrGrey.Height() ) {
        std::cerr << "warning: Camera model and captured image's height do not match. Are you using the correct CMod file?" << std::endl;
    }
    if( m_nImageWidth != m_CModPyrGrey.Width() ) {
        std::cerr << "warning: Camera model and captured image's width do not match. Are you using the correct CMod file?" << std::endl;
    }

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
    if( m_pMap->GetNumFrames() == 0 ) {

        // nope, so set path base pose and global pose accordingly
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

        // since this is a new keyframe, reset last pose
        m_dLastEstimate.setIdentity();
    }

    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// this is the main entry point that the enclosing application calls to advance the engine forward
bool DenseFrontEnd::Iterate(
        const CamImages&    vImages     //< Input: Camera capture
    )
{
    // get GUI variables
    pangolin::Var<float>            ui_fRMSE("ui.RMSE");

    // update error level in case user changed it
    mvl::PrintHandlerSetErrorLevel( feConfig.g_nErrorLevel );

    // given a motion model or IMU, get estimated pose
    // based on this pose, load closest keyframe (euclidean distance including rotation of sorts)

    // run ESM to localize

    // drop estimate into path vector

    // update global pose

    // check point threshold to see if new keyframe must be added to the map

    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //

    // FOR NOW
    // run ESM between current and previous frame

    // adjust Tpc to our previous estimate from keyframe
    Eigen::Matrix4d Tpc = m_dLastEstimate;

    // use constant velocity motion model??
    if( feConfig.g_bConstantVelocityMotionModel == true ) {
        if( m_pMap->GetTransformFromParent( m_pCurFrame->GetId(), Tpc ) == true ) {
            Tpc = m_dLastEstimate * Tpc;
        }
    }

    unsigned int nNumObservations;
    double dTrackingError = _EstimateRelativePose( vImages[0].Image, m_pMap->GetCurrentKeyframe(), Tpc, nNumObservations );
    ui_fRMSE = dTrackingError;

    std::cout << "Estimate was: " << mvl::T2Cart(Tpc).transpose() << std::endl << std::endl;

    if( dTrackingError < 5 ) {
        m_eTrackingState = eTrackingGood;
    } else if( dTrackingError < 20 ) {
        m_eTrackingState = eTrackingPoor;
    } else if( dTrackingError < 35 ) {
        std::cerr << "warning: tracking is bad. (RMSE: " << dTrackingError << ")" << std::endl;
        m_eTrackingState = eTrackingBad;
    } else {
        std::cerr << "warning: tracking failed. (RMSE: " << dTrackingError << ")" << std::endl;
        m_eTrackingState = eTrackingFail;
        return false;
    }

    // update global pose
    m_dGlobalPose = m_dGlobalPose * Tpc;

    // TODO get this from the camera directly.. the property map should have it
    double dSensorTime = mvl::Tic();

    // drop new keyframe if number of observations is too low
    if( nNumObservations < (feConfig.g_fKeyframePtsThreshold * m_nImageHeight * m_nImageWidth) ) {

        // allocate thumb images
        cv::Mat GreyThumb( m_nThumbHeight, m_nThumbWidth, CV_8UC1 );
        cv::Mat DepthThumb( m_nThumbHeight, m_nThumbWidth, CV_32FC1 );

        // generate thumbnails
        GenerateGreyThumbnail( m_cdTemp, vImages[0].Image, GreyThumb );
        GenerateDepthThumbnail( m_cdTemp, vImages[1].Image, DepthThumb );

        FramePtr pNewKeyframe = m_pMap->NewKeyframe( dSensorTime, vImages[0].Image, vImages[1].Image, GreyThumb, DepthThumb );

        if( pNewKeyframe == NULL ) {
            std::cerr << "error: generating new keyframe." << std::endl;
            return false;
        }

        // link previous frame with new frame (relative pose)
        m_pMap->LinkFrames( m_pCurFrame, pNewKeyframe, mvl::TInv(m_dLastEstimate) * Tpc );

        // update current frame
        m_pCurFrame = pNewKeyframe;

        // set new keyframe as current keyframe
        m_pMap->SetKeyframe( m_pCurFrame );

        // since this is a new keyframe, reset last pose
        m_dLastEstimate.setIdentity();

    } else {

        // allocate thumb image
        cv::Mat GreyThumb( m_nThumbHeight, m_nThumbWidth, CV_8UC1 );

        // generate thumbnail
        GenerateGreyThumbnail( m_cdTemp, vImages[0].Image, GreyThumb );

        // otherwise drop a regular frame
        FramePtr pNewFrame = m_pMap->NewFrame( dSensorTime, vImages[0].Image, GreyThumb );

        if( pNewFrame == NULL ) {
            std::cerr << "error: generating new frame." << std::endl;
            return false;
        }

        // link previous frame with new frame (relative pose)
        m_pMap->LinkFrames( m_pCurFrame, pNewFrame, mvl::TInv(m_dLastEstimate) * Tpc );

        // update last pose as a cummulative pose
        m_dLastEstimate = m_dLastEstimate * Tpc;

        // update current frame
        m_pCurFrame = pNewFrame;

    }


    // check for loop closure
    // TODO update this to reflect the new changes in keyframes and regular frames
    // I assume the LC can only happen against keyframes, so validate that?
    double          dLoopClosureError;
    Eigen::Matrix4d LoopClosureT;
    int nLoopClosureFrameId = _LoopClosure( m_pCurFrame, LoopClosureT, dLoopClosureError );
    if( nLoopClosureFrameId != -1 && dLoopClosureError < feConfig.g_dLoopClosureThreshold ) {
        std::cout << "Loop Closure Detected!!! Frame: " << nLoopClosureFrameId << " Error: " << dLoopClosureError << std::endl;
        m_eTrackingState = eTrackingLoopClosure;

        // link frames
        // TODO Ask Steeeeeeevveeeee!! Why the hell is this weird behaviour happening? Comment the top line
        // and uncomment the bottom, and no loop closure is found.. WTF?!?!?!?
        m_pMap->LinkFrames( m_pMap->GetFramePtr(nLoopClosureFrameId), m_pCurFrame, LoopClosureT );
//        m_pMap->LinkFrames( m_pCurKeyframe, m_pMap->GetFramePtr(nLoopClosureFrameId),  LoopClosureT.inverse() );
    }


    return true;
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


    //--- upload GREYSCALE data to GPU as a pyramid
    m_cdGreyPyr[0].MemcpyFromHost( GreyImg.data, m_nImageWidth );
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


    //--- localize
    // IMPORTANT!!!!!!!
    // The localization code operates in VISION frame. Thus, we convert our initial pose from ROBOTICS to VISION
    // and once the solution is found, we convert it from VISION to ROBOTICS.

    // vision-robotics permutation matrix
    Eigen::Matrix4d Tvr;
    Tvr << 0, 1, 0, 0,
           0, 0, 1, 0,
           1, 0, 0, 0,
           0, 0, 0, 1;

    // convert initial pose to VISION reference frame
    Tkc = Tvr * Tkc * Tvr.inverse();

    double          dLastError;

    for( int PyrLvl = MAX_PYR_LEVELS-1; PyrLvl >= 0; PyrLvl-- ) {
        dLastError = DBL_MAX;
        for(int ii = 0; ii < feConfig.g_vPyrMaxIters[PyrLvl]; ii++ ) {
            const unsigned              PyrLvlWidth = m_nImageWidth >> PyrLvl;
            const unsigned              PyrLvlHeight = m_nImageHeight >> PyrLvl;

            Eigen::Matrix3d             Kg = m_CModPyrGrey.K( PyrLvl );     // grey sensor's instrinsics
            Eigen::Matrix3d             Kd = m_CModPyrDepth.K( PyrLvl );    // depth sensor's intrinsics
            Eigen::Matrix4d             Tgd = m_CModPyrDepth.GetPose();     // depth sensor's pose w.r.t. grey sensor
            Eigen::Matrix4d             Tck = Tkc.inverse();
            Eigen::Matrix<double,3,4>   KgTck = Kg * Tck.block<3,4>(0,0);   // precompute for speed

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

    // convert estimate to ROBOTICS frame
    Tkc = Tvr.inverse() * Tkc * Tvr;

    return dLastError;
}


// TODO put this somewhere else
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SAD score image1 with image2 -- images are assumed to be same type & dimensions
// returns: SAD score
template < typename T >
inline float ScoreImages(
        const cv::Mat&              Image1,
        const cv::Mat&              Image2
        )
{
    float fScore = 0;
    for( int ii = 0; ii < Image1.rows; ii++ ) {
        for( int jj = 0; jj < Image1.cols; jj++ ) {
            fScore += fabs(Image1.at<T>(ii, jj) - Image2.at<T>(ii, jj));
        }
    }
    return fScore;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int DenseFrontEnd::_LoopClosure(
        FramePtr                pFrame,         //< Input: Frame we are attempting to find a loop closure
        Eigen::Matrix4d&        T,              //< Output: Transform between input frame and closest matching frame
        double&                 dError          //< Output: RMSE of estimated transform
    )
{
    // TODO use frame's global pose to find closest frames and from there do DFS? some sort of Kd-tree?
    // this is only for speed-up purposes to achieve near RT loop closure

    // TODO add a vector of "top" matches so we can compare with multiple frames instead of just 1
//    std::vector< FramePtr >     vMatches;

    // TODO add grey-depth contribution ratio for matching

    float       fBestScore = FLT_MAX;
    FramePtr    pBestMatch;

    for( int ii = 0; ii < m_pMap->GetNumFrames(); ii++ ) {

        // do not try to compare with frames too close too us
        if( abs(ii - pFrame->GetId()) <= feConfig.g_nLoopClosureMargin ) {
            continue;
        }

        // get frame pointer
        FramePtr pMatchFrame = m_pMap->GetFramePtr( ii );

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

    // if no match is found, return -1
    if( fBestScore > (feConfig.g_nLoopClosureSAD * (m_nThumbHeight * m_nThumbWidth)) ) {
        return -1;
    }

    std::cout << "Found a candidate with score: " << fBestScore << std::endl;

    // calculate transform between best match and input frame
    unsigned int nNumObservations;
    dError = _EstimateRelativePose( pFrame->GetGreyImageRef(), pBestMatch, T, nNumObservations );

    return pBestMatch->GetId();
}
