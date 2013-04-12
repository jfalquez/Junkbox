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
    m_nThumbHeight = m_nImageHeight >> (MAX_PYR_LEVELS-1);
    m_nThumbWidth = m_nImageWidth >> (MAX_PYR_LEVELS-1);

    // initialize max number of iterations to perform at each pyramid level
    // level 0 is finest (ie. biggest image)
    feConfig.g_vPyrMaxIters.resize( MAX_PYR_LEVELS );
    feConfig.g_vPyrMaxIters.setZero();
    feConfig.g_vPyrMaxIters << 1, 1, 2, 2, 5;

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


    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///


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

        FramePtr pFrame = m_pMap->NewKeyframe( dSensorTime, vImages[0].Image, vImages[1].Image, GreyThumb, DepthThumb );

        if( pFrame == NULL ) {
            return false;
        }
        m_pMap->SetKeyframe( pFrame );
    }

    // reset Tpc
    m_T_p_c.setIdentity();

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

    // The map's global poses are updated at the end of every iteration with respect to the last frame.
    // So in essence, the previous frame is the new origin in global coordinates: Twp = I4
    Eigen::Matrix4d T_w_c; T_w_c.setIdentity();
    Eigen::Matrix4d T_w_p; T_w_p.setIdentity();


    ///---------- MOTION MODELS
    // given a motion model or IMU, get estimated pose
    if( feConfig.g_bConstantVelocityMotionModel == true ) {
        T_w_c = m_T_p_c;
    }


    ///---------- CREATE NEW FRAME WITH INPUT IMAGES
    // allocate thumb image
    cv::Mat GreyThumb( m_nThumbHeight, m_nThumbWidth, CV_8UC1 );

    // generate thumbnail
    GenerateGreyThumbnail( m_cdTemp, vImages[0].Image, GreyThumb );

    // TODO get this from the camera directly.. the property map should have it
    double dSensorTime = mvl::Tic();

    // create frame
    FramePtr pFrame = m_pMap->NewFrame( dSensorTime, vImages[0].Image, GreyThumb );
    if( pFrame == NULL ) {
        std::cerr << "error: generating new frame." << std::endl;
        return false;
    }
    const unsigned int nFrameId = pFrame->GetId();


    ///---------- LOCALIZE AGAINST LAST KEYFRAME
    Tic("Localize Last");
    // get path of global poses
    std::map< unsigned int, Eigen::Matrix4d >& vPath = m_pMap->GetInternalPath();

    FramePtr pLastKeyframe = m_pMap->GetCurrentKeyframe();
    const unsigned int nLastKeyframeId = pLastKeyframe->GetId();
    Eigen::Matrix4d T_w_lk = vPath[nLastKeyframeId];

    // Tkc is what the estimator will gives us back
    // we can seed this via: Tkc = Tkw * Twc = TInv( Twk ) * Twc
    Eigen::Matrix4d T_lk_c = mvl::TInv( T_w_lk ) * T_w_c;

    unsigned int nNumObsLastKeyframe;
    double dErrorLastKeyframe = _EstimateRelativePose( pFrame->GetGreyImageRef(), pLastKeyframe, T_lk_c, nNumObsLastKeyframe );
    m_Analytics["RMSE Last"] = std::pair<double, double>( dErrorLastKeyframe, 0 );
    {
        Eigen::Vector6d E = mvl::T2Cart(T_lk_c);
        PrintMessage( 1, "--- Estimate (L) (%f): [ %f, %f, %f, %f, %f, %f ]\n", dErrorLastKeyframe, E(0), E(1), E(2), E(3), E(4), E(5) );
    }
    Toc("Localize Last");


    ///---------- FIND CLOSEST KEYFRAMES
    Tic("Find Close KF");
    // based on current pose, load closest keyframe (euclidean distance including rotation of sorts)
    std::vector< std::pair< unsigned int, float > > vNearKeyframes;
    m_pMap->FindClosestKeyframes( T_w_c, feConfig.g_fCloseKeyframeNorm, vNearKeyframes );


    ///---------- FIND BEST KEYFRAME
    FramePtr pClosestKeyframe = NULL;
    if( vNearKeyframes.empty() == false ) {

        // TODO add grey-depth contribution ratio for matching?
        float       fBestScore = FLT_MAX;

        for( unsigned int ii = 0; ii < vNearKeyframes.size(); ++ii ) {

            const unsigned int nId = std::get<0>( vNearKeyframes[ii] );

            // get frame pointer
            FramePtr pCandidateFrame = m_pMap->GetFramePtr( nId );

            // do not compare with non-keyframes
            if( pCandidateFrame->IsKeyframe() == false ) {
                continue;
            }

            float fScore = ScoreImages<unsigned char>( pFrame->GetGreyThumbRef(), pCandidateFrame->GetGreyThumbRef() );

            if( fScore < fBestScore ) {
                fBestScore = fScore;
                pClosestKeyframe = pCandidateFrame;
            }
        }

        double dSADThreshold = feConfig.g_nLoopClosureSAD * (m_nThumbHeight * m_nThumbWidth);

        m_Analytics["LC SAD"] = std::pair<double, double>( fBestScore, dSADThreshold );

        // if match is found, find pose estimate
        if( fBestScore > dSADThreshold ) {
            pClosestKeyframe = NULL;
        }
    }
    Toc("Find Close KF");


    ///---------- LOCALIZE AGAINST CLOSEST KEYFRAME
    Tic("Localize Close");
    unsigned int nClosestKeyframeId;
    unsigned int nNumObsClosestKeyframe;
    double dErrorClosestKeyframe = DBL_MAX;
    Eigen::Matrix4d T_ck_c;
    if( pClosestKeyframe == NULL || pLastKeyframe == pClosestKeyframe ) {
        m_Analytics["RMSE Closest"] = std::pair<double, double>( 0, 0 );
    } else {
        nClosestKeyframeId = pClosestKeyframe->GetId();
        T_ck_c.setIdentity();    // the drift would kill this if we try to seed like for LastKeyframe
        Eigen::Vector6d E2 = mvl::T2Cart(T_ck_c);
        PrintMessage( 1, "--- Seed: [ %f, %f, %f, %f, %f, %f ]\n", E2(0), E2(1), E2(2), E2(3), E2(4), E2(5) );
        dErrorClosestKeyframe = _EstimateRelativePose( pFrame->GetGreyImageRef(), pClosestKeyframe, T_ck_c, nNumObsClosestKeyframe );
        m_Analytics["RMSE Closest"] = std::pair<double, double>( dErrorClosestKeyframe, 0 );
        {
            Eigen::Vector6d E = mvl::T2Cart(T_ck_c);
            PrintMessage( 1, "--- Estimate (C) (%f): [ %f, %f, %f, %f, %f, %f ]\n", dErrorClosestKeyframe, E(0), E(1), E(2), E(3), E(4), E(5) );
        }

        ///---------- CHECK FOR LOOP CLOSURE
        Tic("Loop Closure");
        if( abs(nFrameId - nClosestKeyframeId) > feConfig.g_nLoopClosureMargin ) {
            // frame is "distant" enough that we might consider it a valid loop closure
            if( dErrorLastKeyframe < feConfig.g_dLoopClosureThreshold && dErrorClosestKeyframe < feConfig.g_dLoopClosureThreshold ) {
                PrintMessage( 1, "Loop Closure Detected!!! Current Frame: %d -- Matching Frame: %d (RMSE: %f)\n", nFrameId, nClosestKeyframeId, dErrorClosestKeyframe );
                m_eTrackingState = eTrackingLoopClosure;

                // link frames
                m_pMap->LinkFrames( pLastKeyframe, pFrame, T_lk_c );
                m_pMap->LinkFrames( pClosestKeyframe, pFrame, T_ck_c );

                // reset m_Tpc in order to discard accumulated drift (for constant velocity model)
                m_T_p_c.setIdentity();

                // update internal path
                m_pMap->UpdateInternalPath();

                Toc("Loop Closure");
                Toc("Localize Close");
                return true;
            }
        }
        Toc("Loop Closure");
    }
    Toc("Localize Close");


    ///---------- NORMAL TRACKING
    // choose between best pose estimates
    FramePtr pKeyframe;
    unsigned int nNumObservations;
    double dTrackingError;
    Eigen::Matrix4d T_k_c;
    Eigen::Matrix4d T_w_k;
    if( dErrorLastKeyframe < dErrorClosestKeyframe || feConfig.g_bAlwaysUseLastKeyframe ) {
        // choose estimate based on last keyframe
        pKeyframe = pLastKeyframe;
        dTrackingError = dErrorLastKeyframe;
        nNumObservations = nNumObsLastKeyframe;
        T_k_c = T_lk_c;
        T_w_k = T_w_lk;
    } else {
        // choose estimate based on closest keyframe
        pKeyframe = pClosestKeyframe;
        dTrackingError = dErrorClosestKeyframe;
        nNumObservations = nNumObsClosestKeyframe;
        T_k_c.setIdentity();
        T_w_k.setIdentity();
    }

    if( dTrackingError < 8.0 ) {
        m_eTrackingState = eTrackingGood;
    } else if( dTrackingError < 20.0 ) {
        m_eTrackingState = eTrackingPoor;
    } else if( dTrackingError < 35.0 ) {
        PrintMessage( 0, "warning: tracking is bad. (RMSE: %f)\n", dTrackingError );
        m_eTrackingState = eTrackingBad;
    } else {
        PrintMessage( 0, "warning: tracking is failing (RMSE: %f)!! Calling relocalizer... \n", dTrackingError );

        ///---------- TRY GLOBAL RELOCALIZER
        // TODO?? Expand search??


        ///---------- WHEN ALL FAILS
        PrintMessage( 1, "critical: Relocalizer failed!\n" );
        m_eTrackingState = eTrackingFail;
        // discard estimate
        T_k_c.setIdentity();
    }


    ///---------- SAVE ESTIMATE
    m_pMap->LinkFrames( pKeyframe, pFrame, T_k_c );


    ///---------- UPDATE KEYFRAME
    m_pMap->SetKeyframe( pKeyframe );


    ///---------- KEYFRAME GENERATION
    const unsigned int nNumDepthPts = cv::countNonZero( vImages[1].Image );
    double dObsPts = (double)nNumObservations / (double)nNumDepthPts;

    // convert frame to keyframe if number of observations is too low
    m_Analytics["Num Obs"] = std::pair<double, double>( dObsPts, feConfig.g_fKeyframePtsThreshold );
    if( dObsPts < feConfig.g_fKeyframePtsThreshold ) {

        // allocate thumb images
        cv::Mat DepthThumb( m_nThumbHeight, m_nThumbWidth, CV_32FC1 );

        // generate thumbnails
        GenerateDepthThumbnail( m_cdTemp, vImages[1].Image, DepthThumb );

        pFrame->SetKeyframeFlag();
        pFrame->SetDepthImage( vImages[1].Image );
        pFrame->SetDepthThumb( DepthThumb );

        m_pMap->SetKeyframe( pFrame );

    }


    // we would also like to get Tpc: transform from previous frame to current frame
    // we obtain this via: Tpc = Tpk * Tkc
    // given: Tpk = Tpw * Twk = TInv( Twp ) * Twk
    m_T_p_c = mvl::TInv( T_w_p ) * T_w_k * T_k_c;


    // update internal path
    m_pMap->UpdateInternalPath();

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


    //--- upload GREYSCALE data to GPU as a pyramid
    m_cdGreyPyr[0].MemcpyFromHost( lGreyImg.data, m_nImageWidth );
    m_cdKeyGreyPyr[0].MemcpyFromHost( pKeyframe->GetGreyImagePtr() );

    for( unsigned int ii = 0; ii < ui_nBlur; ++ii ) {
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
                                                                                    fNormC, ui_bDiscardMaxMin, 0.3, 60.0 );

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
                    PrintMessage( feConfig.g_nDebugGN, "warning(@L%d I%d) LS trashed. Rank deficient!\n", PyrLvl+1, ii+1 );
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
                    PrintMessage( feConfig.g_nDebugGN, "warning(@L%d I%d) LS trashed. Rank deficient!\n", PyrLvl+1, ii+1 );
                    continue;
                }

                Eigen::Vector3d rX;
                rX = - (lu_JTJ.solve(rRHS));

                // pack solution
                X.setZero();
                X.tail(3) = rX;
            }

            // everything seems fine... apply update
            Tkc = (Tkc.inverse() * Sophus::SE3::exp(X).matrix()).inverse();

            const double dNewError = sqrt(LSS.sqErr / LSS.obs);

            // if error decreases too slowly, break out of this level
            if( ( fabs( dNewError - dLastError ) < ui_fBreakErrorThreshold ) && ui_bBreakEarly ) {
                PrintMessage( feConfig.g_nDebugGN, "notice: Breaking early @ L%d : I%d ...\n", PyrLvl+1, ii+1 );
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
