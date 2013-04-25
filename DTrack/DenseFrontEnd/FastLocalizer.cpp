#include <Mvlpp/Mvl.h>

#include <pangolin/pangolin.h>
#include <cuda_gl_interop.h>

#include <Utils/ImageHelpers.h>

#include "FastLocalizer.h"


// Global CVars
FastLocalizerConfig         flConfig;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
FastLocalizer::FastLocalizer( unsigned int nImageWidth, unsigned int nImageHeight ) :
    m_cdGreyPyr( nImageWidth, nImageHeight ),
    m_cdKeyGreyPyr( nImageWidth, nImageHeight ),
    m_cdKeyDepthPyr( nImageWidth, nImageHeight ),
    m_cdWorkspace( nImageWidth * sizeof(Gpu::LeastSquaresSystem<float,6>), nImageHeight ),
    m_cdDebug( nImageWidth, nImageHeight ),
    m_cdTemp( nImageWidth, nImageHeight )
{
    mvl::PrintHandlerSetErrorLevel( flConfig.g_nErrorLevel );

    m_nImageWidth   = nImageWidth;
    m_nImageHeight  = nImageHeight;

    // calculate thumb image dimensions
    m_nThumbHeight = m_nImageHeight >> (MAX_PYR_LEVELS-1);
    m_nThumbWidth = m_nImageWidth >> (MAX_PYR_LEVELS-1);

    // initialize max number of iterations to perform at each pyramid level
    // level 0 is finest (ie. biggest image)
    flConfig.g_vPyrMaxIters.resize( MAX_PYR_LEVELS );
    flConfig.g_vPyrMaxIters.setZero();
    flConfig.g_vPyrMaxIters << 1, 2, 3, 4, 5;

    // initialize if full estimate should be performed at a particular level
    // 1: full estimate          0: just rotation
    flConfig.g_vPyrFullMask.resize( MAX_PYR_LEVELS );
    flConfig.g_vPyrFullMask.setZero();
    flConfig.g_vPyrFullMask << 1, 1, 1, 1, 0;

    m_pMap   = NULL; // passed in by the user
    m_pTimer = NULL; // passed in by the user

    // check CUDA
    CheckMemoryCUDA();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
FastLocalizer::~FastLocalizer()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool FastLocalizer::Init(
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
    std::cout << "Live Greyscale Camera Intrinsics: " << std::endl;
    std::cout << m_pMap->GetLiveGreyCameraK() << std::endl << std::endl;
    std::cout << "Reference Greyscale Camera Intrinsics: " << std::endl;
    std::cout << m_pMap->GetRefGreyCameraK() << std::endl << std::endl;
    std::cout << "Reference Depth Camera Intrinsics: " << std::endl;
    std::cout << m_pMap->GetRefDepthCameraK() << std::endl << std::endl;
    std::cout << "Tgd: " << std::endl;
    std::cout << mvl::T2Cart(m_pMap->GetRefDepthCameraPose()).transpose() << std::endl << std::endl;


    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///

    // allocate thumb image
    cv::Mat GreyThumb( m_nThumbHeight, m_nThumbWidth, CV_8UC1 );

    // generate thumbnail
    GenerateGreyThumbnail( m_cdTemp, vImages[0].Image, GreyThumb );

    // find most similar keyframes
    std::vector< std::pair< unsigned int, float > > vSimilarKeyframes;
    const float fMaxSAD =  (m_nThumbHeight * m_nThumbWidth) * 10;
    m_pMap->FindSimilarKeyframes( GreyThumb, fMaxSAD, vSimilarKeyframes );

    double dLastError = DBL_MAX;
    Eigen::Matrix4d Tkc;
    for( unsigned int ii = 0; ii < vSimilarKeyframes.size(); ii++ ) {

        // get frame pointer
        const unsigned int nId = std::get<0>( vSimilarKeyframes[ii] );
        FramePtr pSimilarKeyframe = m_pMap->GetFramePtr( nId );

        Eigen::Matrix4d T;
        T.setIdentity();
        unsigned int nNumObs;
        double dError = _EstimateRelativePose( vImages[0].Image, pSimilarKeyframe, T, nNumObs );

        if( dError > dLastError ) {
            break;
        }

        Tkc = T;
        dLastError = dError;
        m_pMap->SetKeyframe( nId );
    }

    FramePtr pKeyframe = m_pMap->GetCurrentKeyframe();
    const unsigned int nKeyframeId = pKeyframe->GetId();

    std::cout << "Best Keyframe: " << nKeyframeId << std::endl;
    Eigen::Vector6d E;
    E = mvl::T2Cart( Tkc );
    std::cout << "Esimate: " << E.transpose() << std::endl;

    // get path of global poses
    std::map< unsigned int, Eigen::Matrix4d >& vPath = m_pMap->GetInternalPath();

    Eigen::Matrix4d Twk = vPath[nKeyframeId];

    // store estimated pose in global coordinate frame
    m_pMap->m_dCurPose = m_pMap->m_dPrevPose = Twk * Tkc;

    /// for each candidate
    /// run coarse pyramid and discard if error > THRESHOLD
    /// run next level and discard if error > THRESHOLD
    /// ...
    /// in the end, chose one with lowest score

    /// for now, do this inneficiently


    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool FastLocalizer::Iterate(
        const CamImages&    vImages     //< Input: Camera capture
    )
{
    // update error level in case user changed it
    mvl::PrintHandlerSetErrorLevel( flConfig.g_nErrorLevel );

    Eigen::Matrix4d& T_w_c = m_pMap->m_dCurPose;
    Eigen::Matrix4d& T_w_p = m_pMap->m_dPrevPose;

    ///---------- MOTION MODELS
    // given a motion model or IMU, get estimated pose
    if( flConfig.g_bConstantVelocityMotionModel == true ) {
        Eigen::Matrix4d T_p_c = mvl::TInv( T_w_p ) * T_w_c;
        T_w_c = T_w_c * T_p_c;
    }

    ///---------- CREATE NEW FRAME WITH INPUT IMAGES
    // allocate thumb image
    cv::Mat GreyThumb( m_nThumbHeight, m_nThumbWidth, CV_8UC1 );

    // generate thumbnail
    GenerateGreyThumbnail( m_cdTemp, vImages[0].Image, GreyThumb );


    ///---------- LOCALIZE AGAINST LAST KEYFRAME
    Tic("Localize Last");
    // get path of global poses
    std::map< unsigned int, Eigen::Matrix4d >& vPath = m_pMap->GetInternalPath();

    FramePtr pLastKeyframe = m_pMap->GetCurrentKeyframe();
    const unsigned int nLastKeyframeId = pLastKeyframe->GetId();
    Eigen::Matrix4d T_w_lk = vPath[nLastKeyframeId];

    // Tkc is what the estimator will gives us back
    // we can seed this via: Tkc = Tkw * Twc = TInv( Twk ) * Twc
    Eigen::Matrix4d T_lk_c;
    if( T_w_c != Eigen::Matrix4d::Identity() ) {
        T_lk_c = mvl::TInv( T_w_lk ) * T_w_c;
    } else {
        T_lk_c.setIdentity();
    }
    T_lk_c.setIdentity();
    {
        Eigen::Vector6d E2 = mvl::T2Cart(T_lk_c);
        PrintMessage( 1, "--- Seed: [ %f, %f, %f, %f, %f, %f ]\n", E2(0), E2(1), E2(2), E2(3), E2(4), E2(5) );
    }

    unsigned int nNumObsLastKeyframe;
    double dErrorLastKeyframe = _EstimateRelativePose( vImages[0].Image, pLastKeyframe, T_lk_c, nNumObsLastKeyframe );
    m_Analytics["RMSE Last"] = std::pair<double, double>( dErrorLastKeyframe, 0 );
    {
        Eigen::Vector6d E = mvl::T2Cart(T_lk_c);
        PrintMessage( 1, "--- Estimate (L) (%f): [ %f, %f, %f, %f, %f, %f ]\n", dErrorLastKeyframe, E(0), E(1), E(2), E(3), E(4), E(5) );
    }
    Toc("Localize Last");


    ///---------- FIND CLOSEST KEYFRAMES
    Tic("Find Close KF");
    // based on current pose, load closest keyframe (euclidean distance including rotation of sorts)
    std::vector< std::pair< unsigned int, float > > vClosestKeyframes;
    m_pMap->FindClosestKeyframes( T_w_c, flConfig.g_fCloseKeyframeNorm, vClosestKeyframes );


    ///---------- FIND BEST KEYFRAME
    FramePtr pClosestKeyframe = NULL;
    if( vClosestKeyframes.empty() == false ) {

        // TODO add grey-depth contribution ratio for matching?
        float       fBestScore = FLT_MAX;

        for( unsigned int ii = 0; ii < vClosestKeyframes.size(); ++ii ) {

            // get frame pointer
            const unsigned int nId = std::get<0>( vClosestKeyframes[ii] );
            FramePtr pCandidateFrame = m_pMap->GetFramePtr( nId );

            // do not compare with non-keyframes
            if( pCandidateFrame->IsKeyframe() == false ) {
                continue;
            }

            float fScore = ScoreImages<unsigned char>( GreyThumb, pCandidateFrame->GetGreyThumbRef() );

            if( fScore < fBestScore ) {
                fBestScore = fScore;
                pClosestKeyframe = pCandidateFrame;
            }
        }
    }
    Toc("Find Close KF");


    ///---------- LOCALIZE AGAINST CLOSEST KEYFRAME
    Tic("Localize Close");
    unsigned int nNumObsClosestKeyframe;
    double dErrorClosestKeyframe = DBL_MAX;
    Eigen::Matrix4d T_w_ck;
    Eigen::Matrix4d T_ck_c;
    if( pClosestKeyframe == NULL || pLastKeyframe == pClosestKeyframe ) {
        m_Analytics["RMSE Closest"] = std::pair<double, double>( 0, 0 );
    } else {
        const unsigned int nClosestKeyframeId = pClosestKeyframe->GetId();
        T_w_ck = vPath[nClosestKeyframeId];
        if( T_w_c != Eigen::Matrix4d::Identity() ) {
            T_ck_c = mvl::TInv( T_w_ck ) * T_w_c;
        } else {
            T_ck_c.setIdentity();
        }
        T_ck_c.setIdentity();
        {
            Eigen::Vector6d E2 = mvl::T2Cart(T_ck_c);
            PrintMessage( 1, "--- Seed: [ %f, %f, %f, %f, %f, %f ]\n", E2(0), E2(1), E2(2), E2(3), E2(4), E2(5) );
        }
        dErrorClosestKeyframe = _EstimateRelativePose( vImages[0].Image, pClosestKeyframe, T_ck_c, nNumObsClosestKeyframe );
        m_Analytics["RMSE Closest"] = std::pair<double, double>( dErrorClosestKeyframe, 0 );
        {
            Eigen::Vector6d E = mvl::T2Cart(T_ck_c);
            PrintMessage( 1, "--- Estimate (C) (%f): [ %f, %f, %f, %f, %f, %f ]\n", dErrorClosestKeyframe, E(0), E(1), E(2), E(3), E(4), E(5) );
        }
    }
    Toc("Localize Close");


    ///---------- NORMAL TRACKING
    // choose between best pose estimates
    FramePtr pKeyframe;
    unsigned int nNumObservations;
    double dTrackingError;
    Eigen::Matrix4d T_k_c;
    Eigen::Matrix4d T_w_k;
    if( dErrorLastKeyframe < dErrorClosestKeyframe || flConfig.g_bAlwaysUseLastKeyframe ) {
        // choose estimate based on last keyframe
        pKeyframe = pLastKeyframe;
        dTrackingError = dErrorLastKeyframe;
        nNumObservations = nNumObsLastKeyframe;
        T_k_c = T_lk_c;
        T_w_k = T_w_lk;
        std::cout << "--- Estimate against Last Keyframe chosen." << std::endl;
    } else {
        // choose estimate based on closest keyframe
        pKeyframe = pClosestKeyframe;
        dTrackingError = dErrorClosestKeyframe;
        nNumObservations = nNumObsClosestKeyframe;
        T_k_c = T_ck_c;
        T_w_k = T_w_ck;
        std::cout << "--- Estimate against Closest Keyframe chosen." << std::endl;
    }

    m_Analytics["RMSE"] = std::pair<double, double>( dTrackingError, 35.0 );
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

        T_w_c.setIdentity();
        T_w_p.setIdentity();

        // find most similar keyframes
        std::vector< std::pair< unsigned int, float > > vSimilarKeyframes;
        const float fMaxSAD =  (m_nThumbHeight * m_nThumbWidth) * 10;
        m_pMap->FindSimilarKeyframes( GreyThumb, fMaxSAD, vSimilarKeyframes );

        double dLastError = DBL_MAX;
        for( unsigned int ii = 0; ii < vSimilarKeyframes.size(); ii++ ) {

            // get frame pointer
            const unsigned int nId = std::get<0>( vSimilarKeyframes[ii] );
            FramePtr pSimilarKeyframe = m_pMap->GetFramePtr( nId );

            Eigen::Matrix4d T;
            T.setIdentity();
            unsigned int nNumObs;
            double dError = _EstimateRelativePose( vImages[0].Image, pSimilarKeyframe, T, nNumObs );

            if( dError > dLastError ) {
                break;
            }

            T_k_c = T;
            dLastError = dError;
            m_pMap->SetKeyframe( nId );
        }

        FramePtr pKeyframe = m_pMap->GetCurrentKeyframe();
        const unsigned int nKeyframeId = pKeyframe->GetId();

        std::cout << "Best Keyframe: " << nKeyframeId << std::endl;
        Eigen::Vector6d E;
        E = mvl::T2Cart( T_k_c );
        std::cout << "Esimate: " << E.transpose() << std::endl;

        // get path of global poses
        std::map< unsigned int, Eigen::Matrix4d >& vPath = m_pMap->GetInternalPath();

        T_w_k = vPath[nKeyframeId];
    }


    ///---------- UPDATE KEYFRAME
    std::cout << "New Keyframe ID: " << pKeyframe->GetId() << std::endl;
    m_pMap->SetKeyframe( pKeyframe );


    // update global pose
    T_w_p = T_w_c;
    T_w_c = T_w_k * T_k_c;

    return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void FastLocalizer::GetAnalytics(
        std::map< std::string, std::pair< double, double > >&    mData
    )
{
    mData = m_Analytics;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double FastLocalizer::_EstimateRelativePose(
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
        for(int ii = 0; ii < flConfig.g_vPyrMaxIters[PyrLvl]; ii++ ) {
            const unsigned              PyrLvlWidth = m_nImageWidth >> PyrLvl;
            const unsigned              PyrLvlHeight = m_nImageHeight >> PyrLvl;

            Eigen::Matrix3d             Klg = m_pMap->GetLiveGreyCameraK( PyrLvl ); // grey sensor's instrinsics
            Eigen::Matrix3d             Krg = m_pMap->GetRefGreyCameraK( PyrLvl );  // grey sensor's instrinsics
            Eigen::Matrix3d             Krd = m_pMap->GetRefDepthCameraK( PyrLvl ); // depth sensor's intrinsics
            Eigen::Matrix4d             Tgd = m_pMap->GetRefDepthCameraPose();      // depth sensor's pose w.r.t. grey sensor
            Eigen::Matrix4d             Tck = Tkc.inverse();
            Eigen::Matrix<double,3,4>   KlgTck = Klg * Tck.block<3,4>(0,0);         // precompute for speed

            const float fNormC = ui_fNormC * ( 1 << PyrLvl );

            // build system
            Gpu::LeastSquaresSystem<float,6> LSS = Gpu::PoseRefinementFromDepthESM( m_cdGreyPyr[PyrLvl],
                                                                                    m_cdKeyGreyPyr[PyrLvl],
                                                                                    m_cdKeyDepthPyr[PyrLvl],
                                                                                    Klg, Krg, Krd, Tgd, Tck, KlgTck,
                                                                                    m_cdWorkspace, m_cdDebug.SubImage(PyrLvlWidth, PyrLvlHeight),
                                                                                    fNormC, ui_bDiscardMaxMin, 0.1, 140.0 );

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
            if( flConfig.g_vPyrFullMask(PyrLvl) != 0 ) {
                Eigen::FullPivLU< Eigen::Matrix<double,6,6> >    lu_JTJ(LHS);

                // check degenerate system
                if( lu_JTJ.rank() < 6 ) {
                    PrintMessage( flConfig.g_nDebugGN, "warning(@L%d I%d) LS trashed. Rank deficient!\n", PyrLvl+1, ii+1 );
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
                    PrintMessage( flConfig.g_nDebugGN, "warning(@L%d I%d) LS trashed. Rank deficient!\n", PyrLvl+1, ii+1 );
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
                PrintMessage( flConfig.g_nDebugGN, "notice: Breaking early @ L%d : I%d ...\n", PyrLvl+1, ii+1 );
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
