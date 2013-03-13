/*
   \file FrontEnd.cpp

   This class contains the main slam engine program glue-logic.
*/

#include <FrontEnd/FrontEnd.h>
#include <FrontEnd/FrontEndConfig.h>
#include <FrontEnd/IterativeLinearizedSolver.h>
#include <FrontEnd/LinearSystemForLocalization.h>
#include <FrontEnd/AlternatingBA.h>
#include <FrontEnd/ESM.h>
#include <FrontEnd/Utils.h>
#include <FrontEnd/PatchMatch.h>
#include <FrontEnd/CostFunctions.h>
#include <FrontEnd/StartNewLandmarks.h>
#include <FrontEnd/RANSAC.h>

#include <fstream>
#include <sstream>
#include <math.h> // for log

#include <Eigen/SVD>
#include <bits/stl_map.h>

// global cvars
FrontEndConfig g_FrontEndConfig;

using namespace mvl;
using namespace std;
using namespace boost;

///////////////////////////////////////////////////////////////////////////////
FrontEnd::FrontEnd() 
{

    PrintHandlerSetErrorLevel( g_FrontEndConfig.m_nErrorLevel );

    m_pMap   = NULL; // passed in by the user
    m_pTimer = NULL; // passed in by the user

    // create the feature detector:
    FeatureHandler::Options options;

    options.sFeatureDetector         = g_FrontEndConfig.m_sFeatureDetector;    // (fast [d], brisk )
    options.sFeatureDescriptor       = g_FrontEndConfig.m_sFeatureDescriptor;  // (patch [d], freak, brisk)
    options.freak_bOrientationNormalized = g_FrontEndConfig.m_bUseRotationInvariantDescriptor;
    options.freak_nOctaves           = g_FrontEndConfig.m_uNumPyramidLevels;   // default = 1
    options.freak_fPatternScale      = g_FrontEndConfig.m_fFREAKpatternScale;  // default = 22
    options.fast_nThreshold          = g_FrontEndConfig.m_nFASTthreshold;      // default = 1
    options.brisk_nThreshold         = g_FrontEndConfig.m_nBRISKthreshold;     // default = 15
    options.brisk_nOctaves           = g_FrontEndConfig.m_nBRISKoctaves;       // default = 3
    options.brisk_bRotationInvariant = g_FrontEndConfig.m_bUseRotationInvariantDescriptor; // default=yes
    options.brisk_b3dRefinement      = false;

    // initialize handlers (this takes a while, just done once)
    for(unsigned int ii=0; ii < NUM_CAMERAS; ++ii){
        m_vFeatureHandlers[ii].Init( options );
    }

    // initialize thread pool for multi-threading functions
    if(g_FrontEndConfig.m_uNumThreads < 2) {
        m_pThreadPool0  = NULL;
        m_pThreadPool1  = NULL;
    } 
    else {
        m_pThreadPool0  = new boost::threadpool::pool(2);
        m_pThreadPool1  = NULL;//new boost::threadpool::pool(g_FrontEndConfig.m_uNumThreads);

    }
}

///////////////////////////////////////////////////////////////////////////////
FrontEnd::~FrontEnd() 
{
    if(m_pThreadPool0){
        delete m_pThreadPool0;
    }
    if(m_pThreadPool1){
        delete m_pThreadPool1;
    }
    _Clear();
}

///////////////////////////////////////////////////////////////////////////////
// Given a camera, initialize and reset the slam engine
bool FrontEnd::Init(
        const VehicleConfig& vconfig,
        CamFrames&           frames, 
        Map*                 pMap,
        Timer*               pTimer
        )
{
    // Reset member variables
    _Clear();
    
    //Random Seed ( for repeatability )
    srand( g_FrontEndConfig.m_uRANSACSeed );

    //=========================================================
    // Set map, timer, vehicle configuration and back end
    //=========================================================
    m_pMap = pMap;
    m_pTimer = pTimer;
    m_pTimer->SetWindowSize( g_FrontEndConfig.m_uTimerWindowSize );
    
    m_BackEndOptimizer.SetMapPtr( m_pMap );
    m_BackEndOptimizer.SetVehicleConfiguration( vconfig );

    m_dInlierNoiseError = g_FrontEndConfig.m_dInlierThreshold;
    m_dLearningRate     = 0.9; // slow
    
    for(unsigned int ii=0; ii < NUM_CAMERAS; ++ii) {
        m_VehicleConfig.m_dIntrinsics[ii]  = vconfig.m_dIntrinsics[ii];
        m_VehicleConfig.m_vSensorPoses[ii] = vconfig.m_vSensorPoses[ii];
    }
    
    // Link the map with the work set object
    m_WorkSet.SetMapPtr(m_pMap);

    //=========================================================
    // Load ground truth if we have it
    //=========================================================
    _LoadGroundTruth();

    //=========================================================
    // Process first frame
    //=========================================================

    // Create the first frame in the map
    m_pCurFrame = m_pMap->NewFrame( mvl::Tic() );
    m_dGlobalPose.setIdentity();

    // Allocate the image pyramids
    int nImageWidth  = frames[0].Image.cols;
    int nImageHeight = frames[0].Image.rows;
    
    m_vpCurFeatureImages.resize(NUM_CAMERAS);
    m_vpPrevFeatureImages.resize(NUM_CAMERAS);
    
    for( size_t ii = 0; ii < NUM_CAMERAS; ii++ ){
        m_vpCurFeatureImages[ii] = boost::shared_ptr<FeatureImage>( new FeatureImage );
        m_vpCurFeatureImages[ii]->Alloc( nImageWidth, nImageHeight, g_FrontEndConfig.m_uNumPyramidLevels);
        m_vpPrevFeatureImages[ii] = boost::shared_ptr<FeatureImage>( new FeatureImage );
        m_vpPrevFeatureImages[ii]->Alloc( nImageWidth, nImageHeight, g_FrontEndConfig.m_uNumPyramidLevels );
    }
    
    _ExtractFeatures( frames );

    _StartNewLandmarks();
    
    // Save first keyframe
    /*
    m_Keyframe.Init(m_pMap);
    m_Keyframe.IsKeyframe(Eigen::Matrix4d::Identity(),m_pCurFrame);
    m_Keyframe.SetKeyframe(vImages[0].Image,vImages[1].Image);
     */
    return true;
}

///////////////////////////////////////////////////////////////////////////////
// This is the main entry point that the enclosing application calls to advance
// the slam engine forward.
bool FrontEnd::Iterate(CamFrames& frames)
{
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
    _ExtractFeatures( frames );
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

    if( g_FrontEndConfig.m_bDoBundleAdjustment ) {
        Tic("BA");
        if(m_nFrameIndex > 3) {
            //m_BackEndOptimizer.RefineMap(g_FrontEndConfig.m_uBAWindowSize);
            AlternatingBundleAdjustment( 
                    m_pMap, 
                    g_FrontEndConfig.m_uBAWindowSize,
                    g_FrontEndConfig.m_uBANumIter,
                    m_VehicleConfig,
                    g_FrontEndConfig.m_bUseInverseDepthParameterization
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

    return true;
}

///////////////////////////////////////////////////////////////////////////////
std::map< std::string, float > FrontEnd::SystemStatus()
{
	std::map< std::string, float > status;
	
	status["1. Frame"]                   = m_nFrameIndex;
	status["2. LR-SearchWindow"]         = g_FrontEndConfig.m_uMaxDisparity;
	status["3. Num MIT Matches"]         = m_nNumMITMatches;
	status["4. Num tracked landmarks"]   = m_nNumTrackedLandmarks;
	status["5. Num new landmarks"]       = m_nNumNewLandmarks;
	status["6. Mean tracked length"]     = m_dMeanTrackLength;
	status["7. Distance traveled"]		 = m_dEstDistanceTraveled;
	
	return status;
}

///////////////////////////////////////////////////////////////////////////////
// Returns a map with info of the state of several varibles in the engine
void FrontEnd::GetAnalytics( std::map< std::string, double >& mData )
{
    mData = m_mAnalytics;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4d FrontEnd::GetCurrentPose() 
{
    Eigen::Matrix4d Tab;
    m_pMap->GetTransformFromParent( m_pCurFrame->Id(), Tab );

    return Tab;
}

///////////////////////////////////////////////////////////////////////////////
void  FrontEnd::GetActivePoses( std::map<unsigned int, Eigen::Matrix4d>& Poses )
{
    // this will return all poses (nodes in the graph) that could have changed
    // because of the optimization
    m_pMap->GetPosesToDepth( Poses, g_FrontEndConfig.m_uMatchInTimeWindowSize );
}

///////////////////////////////////////////////////////////////////////////////
const std::vector<cv::KeyPoint> FrontEnd::GetCurrentKeypointsForDisplay( unsigned int uCamera )
{
    return m_vpCurFeatureImages[ uCamera ]->GetKeypoints();
}

///////////////////////////////////////////////////////////////////////////////
const std::vector<Landmark> FrontEnd::GetActiveLandmarks()
{
    //printf("get active landmarks ");
    std::vector<Landmark> vLandmarks;
    
    if( m_pCurFrame ) {
         std::vector<Measurement>& vMeasurements = m_pCurFrame->GetMeasurementsVectorRef();
        for( unsigned int ii = 0; ii < vMeasurements.size(); ++ii ) {
            if( vMeasurements[ii].HasGoodMeasurement(0) || 
                vMeasurements[ii].HasGoodMeasurement(1) ) {
                Landmark& landmark = m_pMap->GetLandmarkRef(vMeasurements[ii]);
                vLandmarks.push_back(landmark);
            }
        }
    }
    
    if( m_pPrevFrame ) {
         std::vector<Measurement>& vMeasurements = m_pPrevFrame->GetMeasurementsVectorRef();
        for( unsigned int ii = 0; ii < vMeasurements.size(); ++ii ) {
            if( vMeasurements[ii].HasGoodMeasurement(0) || 
                vMeasurements[ii].HasGoodMeasurement(1) ) {
                Landmark& landmark = m_pMap->GetLandmarkRef(vMeasurements[ii]);
                vLandmarks.push_back(landmark);
            }
        }
    }
    //printf(" - %lu [done]\n",vLandmarks.size());
    return vLandmarks;
}

///////////////////////////////////////////////////////////////////////////////
/*
com::FeaturePoints FrontEnd::GetCurrentMeasurementsForDisplay(unsigned int uCamera)
{

    assert(uCamera < NUM_CAMERAS);

    com::FeaturePoints            Points;
    com::FeaturePoints::Point2d   Pnt;
    Measurement                   OldMeasurement;

    // get pointer to current measurements
    std::vector<Measurement>& vFeatures = m_pCurFrame->GetMeasurementsVectorRef();

    for( unsigned int ii = 0; ii < vFeatures.size(); ++ii ) {
        com::FeaturePoints::FPoint    FeatPnt;
        FeatPnt.outlier       = !vFeatures[ii].HasGoodMeasurement(0); // TODO FIX ME, left and right are independent
        FeatPnt.m_Measurement = vFeatures[ii];

        if(FeatPnt.outlier) continue;

        if( vFeatures[ii].HasGoodMeasurement(uCamera) ) {

            FeatPnt.scale              = 1 << vFeatures[ii].m_nPyramidLevel;
            FeatPnt.matching_error     = vFeatures[ii].m_dMatchingError[uCamera];
            FeatPnt.reprojection_error = vFeatures[ii].m_dReprojectionError[uCamera];

            // get previous measurement locations
            Landmark& landmark = m_pMap->GetLandmark(vFeatures[ii]);

            if(!landmark.Active() ) continue;
            
            FeatPnt.landmark_ref_id = landmark.Id().m_nRefFrameId;
            FeatPnt.landmark_local_idx = landmark.Id().m_nLandmarkIndex;


            const std::vector<MeasurementId>& vTrack = landmark.GetFeatureTrackRef();
            for(unsigned int jj=0; jj < vTrack.size(); ++jj) {
                if( vTrack[jj].m_nFrameId > m_pCurFrame->Id())
                    break;
                m_pMap->GetMeasurementInFrame( vFeatures[ii], vTrack[jj].m_nFrameId, OldMeasurement );

                if( OldMeasurement.HasGoodMeasurement(uCamera) ) {
                    Pnt.x = (float)OldMeasurement.m_dPixel[uCamera][0];
                    Pnt.y = (float)OldMeasurement.m_dPixel[uCamera][1];
                    FeatPnt.track.push_back(Pnt);
                }
            }
            Points.AddPoint(FeatPnt);
            FeatPnt.track.clear();
        }
    }

    return Points;
}
 */

///////////////////////////////////////////////////////////////////////////////
const std::vector<Measurement> FrontEnd::GetCurrentMeasurements()
{
    return m_pCurFrame->GetMeasurementsVector();
}

///////////////////////////////////////////////////////////////////////////////
void FrontEnd::SaveVehiclePosesToFile() 
{

    unsigned int	nStart,nEnd;
    Eigen::Matrix4d Tab, Tacc;
    Eigen::Matrix<double,6,1> GlobalPose;
    Eigen::Matrix<double,6,1> vCartPos;

    GlobalPose.setZero();

    std::vector<SlamEdgePtr>& vMapEdges = m_pMap->GetEdges();

    ofstream fout;
    fout.open("VehicleTrajectory.txt");

    fout << GlobalPose(0) << "  " << GlobalPose(1) << "  " << GlobalPose(2) << "  ";
    fout << GlobalPose(3) << "  " << GlobalPose(4) << "  " << GlobalPose(5) << endl;

    Tacc = Eigen::Matrix4d::Identity();

    for(unsigned int ii = 0; ii < vMapEdges.size(); ++ii) {

        nStart = vMapEdges[ii]->StartId();
        nEnd   = vMapEdges[ii]->EndId();
        vMapEdges[ii]->GetTransform(nStart,nEnd,Tab);
        Tacc *= Tab;
        //vCartPos = mvl::T2Cart(Tab);
        //GlobalPose += vCartPos;
        vCartPos   = mvl::T2Cart(Tacc);
        GlobalPose = vCartPos;

        fout << GlobalPose(0) << "  " << GlobalPose(1) << "  " << GlobalPose(2) << "  ";
        fout << GlobalPose(3) << "  " << GlobalPose(4) << "  " << GlobalPose(5) << endl;
    }

    fout.close();

}

///////////////////////////////////////////////////////////////////////////////
const char* Pose2Str( const Eigen::Matrix4d& T )
{
    static std::string sStr;
    Eigen::Vector6d x = mvl::T2Cart(T);
    char buf[64];
    snprintf( buf, 64, "[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]'\n", x[0], x[1], x[2], x[3], x[4], x[5] );
    sStr = buf;
    return sStr.c_str();
}

/////////////////////////////////////////////////////////////////////////////// PRIVATE
void FrontEnd::_Clear() 
{
    m_dTrackedFeat			 = 0.0;
    m_dMeanReprojectionError = 0.0;
    m_dEstDistanceTraveled	 = 0.0;
    m_nFrameIndex			 = 0;
    
    m_mAnalytics.clear();
}

///////////////////////////////////////////////////////////////////////////////
void FrontEnd::_SetCurTime( double dCurTime ) 
{
    m_dCurTime = dCurTime;
}

///////////////////////////////////////////////////////////////////////////////
void FrontEnd::_AddNewMeasurementsToMap( std::vector<Measurement>& vMeasurements )
{
    std::vector<Measurement>& rFrameZ = m_pCurFrame->GetMeasurementsVectorRef();
    
    for( unsigned int ii = 0; ii < vMeasurements.size(); ++ii ) {
        
        Measurement& z = vMeasurements[ii];
        
        // add measurement to frame
        z.m_nId.m_nFrameId = m_pCurFrame->Id();
        z.m_nId.m_nLocalIndex = ii;
        rFrameZ.push_back( z );
        
        // add measurement to landmark track
        Landmark& lm = m_pMap->GetLandmarkRef( z );
        bool bSuccessTracking = z.HasGoodMeasurement(0) || z.HasGoodMeasurement(1);
        lm.AddToFeatureTrack( z.m_nId, bSuccessTracking );
    }
}

///////////////////////////////////////////////////////////////////////////////
bool FrontEnd::_GaussNewtonCERES( Eigen::Matrix4d& Tab, std::vector<Measurement>& vMeasurements )
{
    ceres::Problem::Options options_problem;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::HuberLoss loss(1e-4);

//    options_problem.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    options_problem.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    options_problem.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

    ceres::Problem problem(options_problem);
    LocalParamSe3 paramSe3;

    // use sophus to get parametrization
    Eigen::Matrix<double,7,1> T; 
    Eigen::Map<Sophus::SE3d>(T.data()) = Sophus::SE3d(Tab.inverse());  
    problem.AddParameterBlock(T.data(), 7,  &paramSe3);
 
    // add residual blocks
    //std::vector<Measurement>& vMeasurements = CurFramePtr()->GetMeasurementsVectorRef();
    std::vector<Measurement>::iterator itMsr;
    Eigen::Matrix4d Tsb[2];
  
    for( unsigned int ii=0; ii < NUM_CAMERAS; ++ii) { 
        Tsb[ii] =  m_VehicleConfig.m_vSensorPoses[ii].inverse();
    }

    for( itMsr = vMeasurements.begin(); itMsr != vMeasurements.end(); ++itMsr ) {
        Eigen::Vector4d& dX = m_pMap->LandmarkWorkRef( itMsr->m_nId );
        for( unsigned int ii=0; ii < NUM_CAMERAS; ++ii) {
            if(itMsr->HasGoodMeasurement(ii)) {
                PrintMessage( g_FrontEndConfig.m_uDebugCERES, 
                             "CERES: adding cost term for '%s' measured in cam[%d] at [%.2f, %.2f] %f\n",
                             m_pMap->GetLandmarkRef(*itMsr).Name(), ii,
                             itMsr->m_dPixel[ii][0], itMsr->m_dPixel[ii][1], itMsr->m_dReprojectionError[ii] );
                
                ceres::CostFunction* cost = new ceres::AutoDiffCostFunction<ReprojectionError,2,7>(
                        new ReprojectionError(itMsr->m_dPixel[ii][0],
                            itMsr->m_dPixel[ii][1],
                            dX.head(3),
                            Tsb[ii],
                            m_VehicleConfig.m_dIntrinsics[ii],
                            1.0) );

//                ceres::CostFunction* cost = new ceres::AutoDiffCostFunction<ReprojectionErrorC,2,7>(
//                        new ReprojectionErrorC(itMsr->m_dPixel[ii][0],
//                            itMsr->m_dPixel[ii][1],
//                            dX.data(),
//                            Tsb[ii].data(),
//                            m_VehicleConfig.m_dIntrinsics[ii].data(),
//                            1.0) );
                problem.AddResidualBlock(cost,&loss,T.data());
            }
            else{
                PrintMessage( g_FrontEndConfig.m_uDebugCERES, 
                             "CERES: '%s' missing in cam[%d]\n", 
                             m_pMap->GetLandmarkRef(*itMsr).Name(), ii );
            }
        }
    }

    // set solver options
    options.minimizer_progress_to_stdout = false;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.max_num_iterations = g_FrontEndConfig.m_uGaussNewtonMaxNumIter;
    //options.use_block_amd = true;

    Tic( "CERES" );
    ceres::Solve(options, &problem, &summary);
    //std::cout << summary.FullReport() << std::endl;
    Toc( "CERES" );

    Eigen::Map<Sophus::SE3Group<double> > sophusT(T.data());    
    Eigen::Matrix4d Tnew = sophusT.matrix();
    Tab = Tnew.inverse();
    
    PrintMessage( 1, "    CERES estimated pose %s\n", Pose2Str(Tab) );

    return true;

}

///////////////////////////////////////////////////////////////////////////////
bool FrontEnd::_GaussNewton( Eigen::Matrix4d& Tab, std::vector<Measurement>& vMeasurements)
{

    // Get intrinsic and extrinsic cameras' parameters
    std::vector<Eigen::Matrix3d> vK;
    std::vector<Eigen::Matrix4d> vTvs;
    
    for( unsigned int ii=0; ii < NUM_CAMERAS; ++ii ) {
        vK.push_back( m_VehicleConfig.m_dIntrinsics[ii] );   
        vTvs.push_back( m_VehicleConfig.m_vSensorPoses[ii] );   
    }
    
    // initialize problem
    LinearSystemForLocalization problem; 
    
    problem.Init( vK, vTvs, Tab );
            
    // add residual blocks
    for(unsigned int ii = 0; ii < vMeasurements.size(); ++ii) {
        
        Measurement&      msr = vMeasurements[ii];
        Eigen::Vector4d&  X   = m_pMap->LandmarkWorkRef( msr.m_nId );
        
        for( unsigned int cam=0; cam < NUM_CAMERAS; ++cam) {
            if( msr.HasGoodMeasurement(cam)) {
                Eigen::Vector2d z;
                z << msr.m_dPixel[cam][0], msr.m_dPixel[cam][1];
                problem.AddConstraint( X, z, cam );
            }
        }
    }
    
    // solve (IterativeLinearizedSolver.h)
    IterativelySolve( problem, g_FrontEndConfig.m_uGaussNewtonMaxNumIter, 1e-8, 1);
	
    Tab = problem.GetPose();
    
    PrintMessage( 1, "    GaussNewton estimated pose %s\n", Pose2Str(Tab) );
    return true;
	 
}

///////////////////////////////////////////////////////////////////////////////
bool FrontEnd::_RANSAC( Eigen::Matrix4d& Tab, std::vector<Measurement>& vMeasurements ) 
{

    unsigned int      vSubSetIndices[3];
    unsigned int	  nCam;
    Eigen::Matrix3d   vK[NUM_CAMERAS];		// Internal camera parameters
    Eigen::Matrix3d   vKinv[NUM_CAMERAS];   // Inverse internal camera parameters
    Eigen::Matrix4d   vTvs[NUM_CAMERAS];    // Sensor to Vehicle frame
    Eigen::Matrix4d   vTsv[NUM_CAMERAS];	// Vehicle to Sensor frame
    Eigen::Matrix4d   vTws[NUM_CAMERAS];    // Sensor to world transform
    Eigen::Matrix4d   vTsw[NUM_CAMERAS];    // World to sensor transform
    Eigen::Matrix4d   Twv;                  // Vehicle pose in World frame
    Eigen::Vector2d   vZ1,vZ2,vZ3;          // Subset of measurements selected
    Eigen::Vector4d   vX1,vX2,vX3;          // Corresponding landmarks
    Eigen::Matrix3d   M;                    // Permutation matrix 
    Eigen::Matrix4d   Tpose;
    std::vector<Eigen::Matrix4d> vPoses;    // Camera pose solutions for subset

    const unsigned int uMaxTrials        = g_FrontEndConfig.m_uRANSACMaxTrials;
    const unsigned int uMaxDataTrials    = 100; // CVar
    const double       dProb             = g_FrontEndConfig.m_dRANSACProb ;
    const double       dOutlierThreshold = g_FrontEndConfig.m_dRANSACOutlierThreshold;
    const double       dEps              = 1.0e-10;   
    unsigned int       uTrialcount       = 0;
    unsigned int       uNumTrials        = 1;    // Dummy initialization for number of trials.
    double             dFracInliers      = 0.0;
    double             dProbNoOutliers   = 0.0;
    Eigen::Matrix4d    BestTwv           = Eigen::Matrix4d::Identity();

    // Init permutation matrix
    M <<  0.0,0.0,1.0,1.0,0.0,0.0,0.0,1.0,0.0 ;

    // Get valid measurements
    std::vector<Measurement> vValidMeasurements;
    for( unsigned int ii=0; ii < vMeasurements.size(); ++ii ) {
        if( vMeasurements[ii].HasGoodMeasurement(0) || vMeasurements[ii].HasGoodMeasurement(1) ) {
            vValidMeasurements.push_back( vMeasurements[ii] );
        }
    }
    
    const unsigned int uNumMeas  = vValidMeasurements.size();

    // Get inverse camera parameters -- TODO this is unnecessary
    for(unsigned int CAMERA = 0; CAMERA < NUM_CAMERAS; ++CAMERA) {
        vK[CAMERA]    = m_VehicleConfig.m_dIntrinsics[CAMERA];
        vKinv[CAMERA] = vK[CAMERA].inverse();
        vTvs[CAMERA]  = m_VehicleConfig.m_vSensorPoses[CAMERA];
        vTsv[CAMERA]  = vTvs[CAMERA].inverse();
    }

    unsigned int uBestCamera;
    unsigned int uBestNumInliers = 0;
    double       dBestTotalError = DBL_MAX;

    PrintMessage(g_FrontEndConfig.m_uDebugRANSAC,"<RANSAC>\n");
    if( uNumMeas < 3 ){
        PrintMessage( 0, "    RANSAC: not enough measurements\n" );
    }

    unsigned int uTries = 0;
    unsigned int uMinTrials = 10;
    while ( ( (uNumTrials > uTrialcount) || (uTrialcount <= uMinTrials))  && (uNumMeas >= 3)) {
       
        // Select random camera
        nCam = (unsigned int)(rand() % NUM_CAMERAS);
        
        if( !GoodRandIndices( m_pMap, nCam, vSubSetIndices, vValidMeasurements ) ){
            PrintMessage( 0, "    Unable to select a nondegenerate data set.\n" );
            if( uTries++ > uMaxDataTrials ){
                PrintMessage( 0, "    Unable to select a nondegenerate data set after %d tries, bailing\n", uTries );
                return false;
            }
            continue;
        }

        vZ1 << vValidMeasurements[vSubSetIndices[0]].m_dPixel[nCam][0], vValidMeasurements[vSubSetIndices[0]].m_dPixel[nCam][1];
        vZ2 << vValidMeasurements[vSubSetIndices[1]].m_dPixel[nCam][0], vValidMeasurements[vSubSetIndices[1]].m_dPixel[nCam][1];
        vZ3 << vValidMeasurements[vSubSetIndices[2]].m_dPixel[nCam][0], vValidMeasurements[vSubSetIndices[2]].m_dPixel[nCam][1];

        PrintMessage( g_FrontEndConfig.m_uDebugRANSAC,
                     "    Selected Measurements: [%d %d %d] from cam[%d]:"
                     "  [%.3f %.3f],   [%.3f %.3f],   [%.3f %.3f]\n",
                     vSubSetIndices[0],vSubSetIndices[1],vSubSetIndices[2], nCam,
                     vZ1[0], vZ1[1], vZ2[0], vZ2[1], vZ3[0], vZ3[1] );

        // Generate a model from sub-set
        // Get 3D landmarks w.r.t. vehicle
        vX1 = m_pMap->LandmarkWorkRef(vValidMeasurements[vSubSetIndices[0]].m_nId);
        vX2 = m_pMap->LandmarkWorkRef(vValidMeasurements[vSubSetIndices[1]].m_nId);
        vX3 = m_pMap->LandmarkWorkRef(vValidMeasurements[vSubSetIndices[2]].m_nId);   
        
        // Transform 3d points from robotics to cv coordinates, 
        // and divide by last component. This will give us (x,y,z) if we were using 
        // inverse depth parameterization.
        double tmp;
        tmp = vX1(0); vX1(0) = vX1(1)/vX1(3); vX1(1) = vX1(2)/vX1(3); vX1(2) = tmp/vX1(3);
        tmp = vX2(0); vX2(0) = vX2(1)/vX2(3); vX2(1) = vX2(2)/vX2(3); vX2(2) = tmp/vX2(3);
        tmp = vX3(0); vX3(0) = vX3(1)/vX3(3); vX3(1) = vX3(2)/vX3(3); vX3(2) = tmp/vX3(3);
        
        
        // Solve localization problem
        mvl::SolveThreePointPose( vZ1, vZ2, vZ3, vX1.head(3), vX2.head(3), vX3.head(3), vKinv[nCam], vPoses );

        // convert poses back to robot coordinate frame
        for(unsigned int pp=0; pp < vPoses.size(); ++pp){
            Tpose = vPoses[pp];
            //mvl::Vision2AeroInplace(Tpose); <-- BROKEN
            Tpose.block<3,3>(0,0) = M*vPoses[pp].block<3,3>(0,0)*(M.transpose());
            Tpose.block<3,1>(0,3) = M*vPoses[pp].block<3,1>(0,3);
            // convert back to vehicle frame
            vPoses[pp] = Tpose * vTsv[nCam];
        }

        // Find and count inliers for current model
        unsigned int   uPoseNumInliers;
        unsigned int   uBestPose           = 0;
        unsigned int   uBestPoseNumInliers = 0;
        
        double         dError;
        double         dMaxError;
        double         dTotalError;
        double         dBestPoseTotalError = DBL_MAX;

        for(unsigned int pp=0; pp < vPoses.size(); ++pp) {

            Twv = vPoses[pp];

            for(unsigned int CAMERA = 0; CAMERA < NUM_CAMERAS; ++CAMERA) {
                vTws[CAMERA] = Twv * vTvs[CAMERA];
                vTsw[CAMERA] = mvl::TInv(vTws[CAMERA]);
            }

            uPoseNumInliers = 0;
            dTotalError     = 0.0;

            // score the whole set
            for( unsigned int ii = 0; ii < vValidMeasurements.size(); ++ii ) {
                // compute reprojection error in both cameras -- hmm could easily use just one
                dMaxError = 0.0;
                for (unsigned int nCam = 0; nCam < NUM_CAMERAS; ++nCam) {

                    if(!vValidMeasurements[ii].HasGoodMeasurement(nCam)){
                        PrintMessage( g_FrontEndConfig.m_uDebugRANSAC, "    cam[%d] z[%d] -- NO MEASUREMENT\n", nCam, ii );
                        continue;
                    }

                    Eigen::Vector4d  dXwp = m_pMap->LandmarkWorkRef(vValidMeasurements[ii].m_nId);
                    Eigen::Vector2d  dz;
                    Eigen::Vector2d  dPixel;
                    dz << vValidMeasurements[ii].m_dPixel[nCam][0], vValidMeasurements[ii].m_dPixel[nCam][1];

                    // internally the next function uses the inverse of Twv
                    //dPixel = mvl::Project3dTo2dRoboticsFrame( vK[nCam], vTws[nCam], dXwp.head(3));
                    dPixel = Project3dTo2d( dXwp, vTsw[nCam], vK[nCam] );
                    
                    Eigen::Vector2d dErrorVec = dPixel - dz;
                    dError = dErrorVec.norm();

                    Eigen::Vector6d p = mvl::T2Cart( vTws[nCam] );
                    PrintMessage( g_FrontEndConfig.m_uDebugRANSAC,  "Motion est: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] ",
                            p(0), p(1), p(2), p(3), p(4), p(5) );
                    PrintMessage( g_FrontEndConfig.m_uDebugRANSAC, " cam[%d] z[%d] [%.2f, %.2f] hx [%.2f, %.2f] (Error %.3f)\n",
                           nCam, ii, dz(0), dz(1), dPixel(0), dPixel(1), dErrorVec.norm() );
//                    Eigen::Vector6d t = mvl::T2Cart( _TruePose(vTws[0] ) );
//                    printf( "Actual motion   [%.2f, %.2f,%.2f, %.2f,  %.2f, %.2f]\n",
//                            t(0), t(1), t(2), t(3), t(4), t(5) );

                    
                    if(dError > dMaxError){
                        dMaxError = dError;
                    }
                }

                //if "dist" to model is > than the inlier threshold ->FLAG as outlier
                if ( dMaxError < dOutlierThreshold ) {
                    uPoseNumInliers++;
                    dTotalError += dMaxError;
                }else{
                    dTotalError += dOutlierThreshold;
                }
            }
            
            Eigen::Vector6d p = mvl::T2Cart( Twv );
            PrintMessage( 1,  "Motion est: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] ",
                            p(0), p(1), p(2), p(3), p(4), p(5) );
            PrintMessage( 1,  "Inliers: %lu  Error: %.3f ", uPoseNumInliers, dTotalError );
            
//            std::cout << "TESTING Tab: " << std::endl << Twv << std::endl;
//            std::cout << "Num inliers: " << uPoseNumInliers << "  error: " << dTotalError << std::endl;

            if( ( uPoseNumInliers > uBestPoseNumInliers ) || 
                ( uPoseNumInliers == uBestPoseNumInliers && dTotalError < dBestPoseTotalError) ) {
            // CHECK THIS
//            if( ( uPoseNumInliers >= uBestPoseNumInliers ) && (dTotalError < dBestPoseTotalError) ) {
                uBestPose           = pp;
                uBestPoseNumInliers = uPoseNumInliers;
                dBestPoseTotalError = dTotalError;
                PrintMessage( 1,  "(best so far) ");
            } 

            PrintMessage( 1,  "\n");
        }

        // Save bestinliers & update N
        if( ( uBestPoseNumInliers > uBestNumInliers) || 
            ( uBestPoseNumInliers == uBestNumInliers && dBestPoseTotalError < dBestTotalError) ) {  
//        if( ( uBestPoseNumInliers >= uBestNumInliers)  && (dBestPoseTotalError < dBestTotalError) ) {             
            // Largest set of inliers so far (with minimum error)...
            uBestNumInliers = uBestPoseNumInliers;	        // Record data for this model
            dBestTotalError = dBestPoseTotalError;
            BestTwv = vPoses[uBestPose];
            uBestCamera = nCam;
        }

        //Update estimate of N, the number of trials to ensure we pick,
        // with probability p, a data set with no outliers.
        dFracInliers =  ((double)uBestNumInliers)/uNumMeas;
        dProbNoOutliers = 1 -  (dFracInliers*dFracInliers*dFracInliers);
        if ( dProbNoOutliers < dEps ) {
            dProbNoOutliers = dEps;  // Avoid division by -Inf
        }
        if ( dProbNoOutliers > (1-dEps) ) {
            dProbNoOutliers = (1-dEps);  // Avoid division by 0
        }

        uNumTrials = log( 1-dProb ) / log( dProbNoOutliers );
        uTries = 0; // reset the GoodRandIndices counter...
        uTrialcount++;

        // Safeguard vs infinite loop
        if ( uTrialcount > uMaxTrials ) {
            PrintMessage( g_FrontEndConfig.m_uDebugRANSAC, "    RANSAC reached the maximum number of trials\n" );
            break;
        }
    } 	//End Model-est cycle---------------

    //check if a solution was found
    if ( BestTwv == Eigen::Matrix4d::Identity() ) {
        PrintMessage( 0, "    RANSAC did not find a model\n" );
    }else {
        Tab = BestTwv;
    }

    PrintMessage( 1, "    RANSAC found %d inliers with pose est %s\n", uBestNumInliers, Pose2Str(Tab) );

    PrintMessage(g_FrontEndConfig.m_uDebugRANSAC,"<RANSAC>\n");

    return true;
}

///////////////////////////////////////////////////////////////////////////////
// Compute reprojection error std dev.
double FrontEnd::_CalcSigma(
                  const std::vector<Eigen::Matrix3d>&  vK,
                  const std::vector<Eigen::Matrix4d>&  vTws,
                  std::vector<Measurement>&  vMeasurements
                 )
{
    Eigen::Vector4d  dXwp;
    Eigen::Vector2d	 dz;
    Eigen::Vector2d  dPixel;
    std::vector<Eigen::Matrix4d> vTsw;
    unsigned int uNumCameras = vK.size();
    
    for( unsigned int nCam = 0; nCam < uNumCameras; nCam++ ){
        vTsw.push_back( mvl::TInv( vTws[nCam] ) );
    }
    
    std::vector<double> vErrors;
    for( unsigned int nCam = 0; nCam < uNumCameras; nCam++ ){
        for( size_t ii = 0; ii < vMeasurements.size(); ++ii ) {
            Measurement& z = vMeasurements[ii];
            if( z.HasGoodMeasurement(nCam) ){
                // compute reprojection error
                dz << vMeasurements[ii].m_dPixel[nCam][0], vMeasurements[ii].m_dPixel[nCam][1];
                dXwp = m_pMap->LandmarkWorkRef( vMeasurements[ii].m_nId );
                //dPixel = mvl::Project3dTo2dRoboticsFrame( vK[nCam], vTws[nCam], dXwp.head(3) );
                dPixel = Project3dTo2d( dXwp, vTsw[nCam], vK[nCam] );
                vErrors.push_back( (dPixel - dz).squaredNorm() );
            }
        }
    }
    // now get median as our estimate of sigma
    if( !vErrors.empty() ) {
        std::sort( vErrors.begin(), vErrors.end() );
        double dSigma = sqrt(vErrors[vErrors.size()/2]);
        if( dSigma < 0.1 ){ // TODO CVar
            dSigma += 0.1;
        }
        return dSigma;
    }
    return DBL_MAX;
}

///////////////////////////////////////////////////////////////////////////////
// Returns number of inliers
int FrontEnd::_FlagRANSACOutliers(  
         Eigen::Matrix4d&          Tab, 
         std::vector<Measurement>& vMeasurements,
         double&                   dMSRE          // mean squared reprojection error
        ) 
{
    PrintMessage( g_FrontEndConfig.m_uDebugFlagOutliers, "<FlagRANSACOutliers>\n" );
    
    std::vector<Eigen::Matrix3d>  vK(NUM_CAMERAS);
    std::vector<Eigen::Matrix4d>  vTws(NUM_CAMERAS);
    std::vector<Eigen::Matrix4d>  vTsw(NUM_CAMERAS);
    
    int nNumInliers = 0;
    dMSRE = 0.0;

    for(unsigned int CAMERA = 0; CAMERA < NUM_CAMERAS; ++CAMERA) {
        vK[CAMERA]   = m_VehicleConfig.m_dIntrinsics[CAMERA];
        vTws[CAMERA] = Tab * m_VehicleConfig.m_vSensorPoses[CAMERA];
        vTsw[CAMERA] = mvl::TInv( vTws[CAMERA] );
    }
    
    for( size_t ii = 0; ii < vMeasurements.size(); ++ii ) {

        Landmark&    lm  = m_pMap->GetLandmarkRef(vMeasurements[ii]);
        Measurement& msr = vMeasurements[ii];
        
        for (unsigned int nCam = 0; nCam < NUM_CAMERAS; ++nCam) {

            if(!msr.HasGoodMeasurement(nCam)){
                continue;
            }

            // compute reprojection error
            Eigen::Vector2d z( msr.m_dPixel[nCam] );
            Eigen::Vector4d X = m_pMap->LandmarkWorkRef(msr.m_nId);
           // Eigen::Vector2d p = mvl::Project3dTo2dRoboticsFrame( vK[nCam], vTws[nCam], X.head(3));
            Eigen::Vector2d p = Project3dTo2d( X, vTsw[nCam], vK[nCam] );

            Eigen::Vector2d e  = p - z;
            double dSqError	   = e.squaredNorm();
            double dError      = sqrt( dSqError );

            PrintMessage( g_FrontEndConfig.m_uDebugFlagOutliers, 
                         "FlagRANSACOutliers: LM %s cam[%d] z=[%.2f, %.2f], hx=[%.2f, %.2f] error %f\n",
                         lm.Name(), nCam, z[0], z[1], p[0], p[1],  dError );

            msr.m_dReprojectionError[nCam] = dError;

            //if "dist" to model is > than the inlier threshold ->FLAG as outlier
            if( dError > g_FrontEndConfig.m_dRANSACOutlierThreshold ){
                PrintMessage( g_FrontEndConfig.m_uDebugFlagOutliers, 
                             "FlagRANSACOutliers %s reprojection error %f too big\n", lm.Name(), dError );
                msr.Flag( nCam, RMSOutlier );
            }
            else {
                msr.Flag( nCam, GoodMatch );
                dMSRE += dSqError;
                nNumInliers++;
            }
        }
    }

    if(nNumInliers > 0){
        dMSRE /= nNumInliers;
    }else{
        dMSRE = DBL_MAX;
    }

    PrintMessage( g_FrontEndConfig.m_uDebugFlagOutliers, "</FlagRANSACOutliers>\n" );
    
    return nNumInliers;

}

///////////////////////////////////////////////////////////////////////////////
// Returns number of inliers
int FrontEnd::_FlagOutliers(  
         Eigen::Matrix4d&          Tab, 
         std::vector<Measurement>& vMeasurements,
         double&                   dReprojectionError
        ) 
{
    PrintMessage( 1, "<FlagOutliers>\n" );

    
    std::vector<Eigen::Matrix3d>  vK(NUM_CAMERAS);
    std::vector<Eigen::Matrix4d>  vTws(NUM_CAMERAS);
    std::vector<Eigen::Matrix4d>  vTsw(NUM_CAMERAS);
    Eigen::Vector4d               dXwp;
    Eigen::Vector2d	              dz;
    Eigen::Vector2d               dPixel;
    Eigen::Vector2d	              dErrorVec;
    double                        dError;
    int                           nNumInliers = 0;

    dReprojectionError = 0.0;

    for(unsigned int CAMERA = 0; CAMERA < NUM_CAMERAS; ++CAMERA) {
        vK[CAMERA]   = m_VehicleConfig.m_dIntrinsics[CAMERA];
        vTws[CAMERA] = Tab * m_VehicleConfig.m_vSensorPoses[CAMERA];
        vTsw[CAMERA] = mvl::TInv( vTws[CAMERA] );
    }
    
    // get sigma
    double dSigma = _CalcSigma( vK, vTws, vMeasurements );
    
    m_dInlierNoiseError = m_dLearningRate * m_dInlierNoiseError + (1.0 - m_dLearningRate)*dSigma;
    double dOutlierThreshold = 2*m_dInlierNoiseError;
    //double dOutlierThreshold = min( m_dInlierNoiseError * 2, g_FrontEndConfig.m_dInlierThreshold);

    m_mAnalytics["InlierNoise"] = m_dInlierNoiseError;
    
    PrintMessage(1,"FlagOutliers, InlierNoiseError: %0.3f\n",m_dInlierNoiseError);
    // compute reprojection error in the left image
    for( size_t ii = 0; ii < vMeasurements.size(); ++ii ) {

        Landmark &Lmrk = m_pMap->GetLandmarkRef(vMeasurements[ii]);
        
        for (unsigned int nCam = 0; nCam < NUM_CAMERAS; ++nCam) {

            if(!vMeasurements[ii].HasGoodMeasurement(nCam)){
                continue;
            }

            // compute reprojection error
            dz	  << vMeasurements[ii].m_dPixel[nCam][0], vMeasurements[ii].m_dPixel[nCam][1];
            dXwp  = m_pMap->LandmarkWorkRef(vMeasurements[ii].m_nId);

            // internally the next function uses the inverse of Twv
            //dPixel	   = mvl::Project3dTo2dRoboticsFrame( vK[nCam], vTws[nCam], dXwp.head(3));
            dPixel    = Project3dTo2d( dXwp, vTsw[nCam], vK[nCam] );
            dErrorVec = dPixel - dz;
            dError	  = dErrorVec.norm();

            PrintMessage( g_FrontEndConfig.m_uDebugFlagOutliers, 
                         "FlagOutliers: LM %s cam[%d] z=[%.2f, %.2f], hx=[%.2f, %.2f] error %f\n",
                         Lmrk.Name(), nCam, dz[0], dz[1], dPixel[0], dPixel[1],  dError );

            vMeasurements[ii].m_dReprojectionError[nCam] = dError;

            //if "dist" to model is > than the inlier threshold ->FLAG as outlier
            if( dError > dOutlierThreshold ){
                PrintMessage( g_FrontEndConfig.m_uDebugFlagOutliers, 
                             "FlagOutliers %s reprojection error %f too big\n", Lmrk.Name(), dError );
                vMeasurements[ii].Flag( nCam, RMSOutlier );

            }
            else {
                vMeasurements[ii].Flag( nCam, GoodMatch );
                dReprojectionError += dError;
                nNumInliers++;
            }
        }
    }

    if(nNumInliers > 0){
        dReprojectionError /= nNumInliers;
    }
    else{
        dReprojectionError = DBL_MAX;
    }

    PrintMessage( 1, "</FlagOutliers>\n" );
    
    return nNumInliers;

}

///////////////////////////////////////////////////////////////////////////////
// This function will localize a given frame against a reference frame.
bool FrontEnd::_EstimateRelativePose(
        SlamFramePtr pFrameA,
        SlamFramePtr, // pFrameB,
        Eigen::Matrix4d& Tab  //< Output: the estimated transform
        ) 
{

    // Transform 3d position of landmarks with respect to the previous frame
    Tic("ComputeWorkingSet");
    m_WorkSet.ComputeWorkingSet( pFrameA->Id(), g_FrontEndConfig.m_uMatchInTimeWindowSize, m_VehicleConfig );
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
    if( g_FrontEndConfig.m_bDoRANSAC ) {
        _RANSAC( Tab, vNewMeasurements ); 
    }

    int nNumInliers = _FlagRANSACOutliers( Tab, vNewMeasurements, m_dMeanReprojectionError);
    // failure case
    if( nNumInliers < (int)g_FrontEndConfig.m_uRANSACMinInliers ) {
        PrintMessage(0,"RANSAC FAILURE -> Using motion model \n");
        Toc("RANSAC");
        return false;
    }
    Toc("RANSAC");

    Tic("GaussNewton");
    if( g_FrontEndConfig.m_bUseCERES ) {
        _GaussNewtonCERES( Tab, vNewMeasurements );
    } else {
        _GaussNewton( Tab, vNewMeasurements );
    }
    nNumInliers = _FlagOutliers( Tab, vNewMeasurements, m_dMeanReprojectionError );
    m_mAnalytics["MRE"] = m_dMeanReprojectionError;
    Toc("GaussNewton");

    // add new measurements to current frame and to landmark tracks
    _AddNewMeasurementsToMap( vNewMeasurements );
    
    return true;
}

///////////////////////////////////////////////////////////////////////////////
bool FrontEnd::_StartNewLandmarks() 
{
    // function in StartNewLandmarks.h
    StartNewLandmarks( 
            m_pCurFrame,
            m_vpCurFeatureImages,
            m_VehicleConfig,
            m_nNumTrackedLandmarks,
            m_nNumNewLandmarks
           );
    
    // save data for display
    m_mAnalytics["Tracked landmarks"] = (double)m_nNumTrackedLandmarks;
    m_mAnalytics["New landmarks"]     = (double)m_nNumNewLandmarks;
    
    return true;
}

///////////////////////////////////////////////////////////////////////////////
bool FrontEnd::_MatchInTime( Eigen::Matrix4d& Tab, 
                             int nSearchFactor,
                             std::vector<Measurement>& vNewMeasurements ) 
{

    if( m_nFrameIndex == 0 ) {
        PrintMessage(0, "ERROR -> _MatchInTime(): Not enough images for matching");
        exit(0);
    }
    
                                                             
    m_nNumMITMatches = MatchInTime(
                         m_pPrevFrame,  // A
                         m_pCurFrame,  // B
                         m_vpCurFeatureImages, // B
                         m_VehicleConfig.m_vSensorPoses,
                         m_VehicleConfig.m_dIntrinsics,
                         &m_WorkSet,
                         Tab,
                         g_FrontEndConfig.m_uMITSearchWidth * nSearchFactor,
                         g_FrontEndConfig.m_uMITSearchHeight * nSearchFactor,
                         g_FrontEndConfig.m_uMaxDisparity,
                         vNewMeasurements
                        );

    PrintMessage(0,"MATCH-IN-TIME num tentative matches: %d\n", m_nNumMITMatches);
    if( m_nNumMITMatches >=
            g_FrontEndConfig.m_dGoodTrackingPercentage * g_FrontEndConfig.m_uNumFeaturesToTrack  ){
        m_eTrackingState = eTrackingGood;
        return true;
    }
    m_eTrackingState = eTrackingPoor;
    return false;
}

///////////////////////////////////////////////////////////////////////////////
bool FrontEnd::_ReThreading( Eigen::Matrix4d & )
{
    // Get landmarks in working set
    std::vector<Landmark*>&   vLandmarksRef  = m_WorkSet.GetWorkSetLmkRef();

    // Check if we missed landmarks
    std::vector<Landmark*>::iterator itLmk;
    for( itLmk = vLandmarksRef.begin(); itLmk != vLandmarksRef.end(); ++itLmk ) {
        
    } 
    return true;
}

///////////////////////////////////////////////////////////////////////////////
bool FrontEnd::_Relocalization(Eigen::Matrix4d& Tab)
{

    m_pMap->GetTransformFromParent( m_pPrevFrame->Id(), Tab );

    m_pCurFrame->SetBrokenLink();

    return true;
}

///////////////////////////////////////////////////////////////////////////////
// Extract features/Extrac
bool FrontEnd::_ExtractFeatures( CamFrames& frames )
{
    // build the image pyramids
    for( size_t ii = 0; ii < NUM_CAMERAS; ii++ ){
        //m_pThreadPool0->schedule( boost::bind(&FeatureImage::BuildPyramid, 
//                                    boost::ref(m_vpCurFeatureImages[ii]), 
//                                    frames[ii].Image ) );
        m_vpCurFeatureImages[ii]->BuildPyramid( frames[ii].Image );
    }
    //m_pThreadPool0->wait();

    // extract features
    for( size_t ii = 0; ii < NUM_CAMERAS; ii++ ){
       m_pThreadPool0->schedule( boost::bind(&FeatureImage::ExtractFeatures, 
                                 boost::ref(m_vpCurFeatureImages[ii]),
                                 m_vFeatureHandlers[ii] ) );
       
       //m_vpCurFeatureImages[ii]->ExtractFeatures( m_vFeatureHandlers[ii] );
    }
    m_pThreadPool0->wait();
    
    return true;
}

///////////////////////////////////////////////////////////////////////////////
void FrontEnd::_ResetFeatures() 
{
    for(unsigned int ii=0; ii< m_vpCurFeatureImages.size(); ++ii) {
        m_vpCurFeatureImages[ii]->ResetFeatures();
    }
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4d FrontEnd::_TruePose( Eigen::Matrix4d& Tab )
{
    if( !m_vGroundTruth.empty() ){
        Tab = mvl::Cart2T( m_vGroundTruth[ m_nFrameIndex++ ] );
    }
    return Tab;
}

///////////////////////////////////////////////////////////////////////////////
void FrontEnd::_LoadGroundTruth() 
{

    if( g_FrontEndConfig.m_sGroundTruthFile.empty() == false ) {

        ifstream pFile;
        Eigen::Vector6d Pose;

        pFile.open( g_FrontEndConfig.m_sGroundTruthFile.c_str() );
        if( pFile.is_open( ) ) {
            while( !pFile.eof( ) ) {
                pFile >> Pose(0) >> Pose(1) >> Pose(2) >> Pose(3) >> Pose(4) >> Pose(5);
                m_vGroundTruth.push_back( Pose );
            }
        } else {
            PrintMessage(0,"ERROR -> LoadGroundTruth(): opening ground truth file failed");
        }
        pFile.close( );
    }
}

///////////////////////////////////////////////////////////////////////////////
void FrontEnd::_PrintPoseCart(Eigen::Matrix4d& Tab) 
{

   Eigen::Matrix<double,6,1> vP = mvl::T2Cart(Tab);
   cout << setw(8) << "dx:" << setw(13) << vP(0);
   cout << setw(8) << "dy:" << setw(13) << vP(1);
   cout << setw(8) << "dz:" << setw(13) << vP(2);
   cout << setw(10) << "d_roll:" << setw(13) << vP(3)*57.2957796;
   cout << setw(10) << "d_pitch:"<< setw(13) << vP(4)*57.2957796;
   cout << setw(10) << "d_yaw:"  << setw(13) << vP(5)*57.2957796 << endl;
   
}
    
