#include "SensorFusion.h"

/////////////////////////////////////////////////////////////////////////////////////////
SensorFusion::SensorFusion(const int nFilterSize) : m_nFilterSize(nFilterSize)
{

}

/////////////////////////////////////////////////////////////////////////////////////////
PoseData SensorFusion::GetGlobalPose( double dTime )
{
    PoseData PData;

    // check if time is before our history
    if( dTime < m_lPoses.front().m_dTime ) {
        return m_lPoses.front();
    }
    return PData;
}

/////////////////////////////////////////////////////////////////////////////////////////
void SensorFusion::RegisterImuPose( Eigen::Vector3d accel, Eigen::Vector3d gyro, double time )
{
    RegisterImuPose( accel(0), accel(1), accel(2), gyro(0), gyro(1), gyro(2), time );
}


/////////////////////////////////////////////////////////////////////////////////////////
void SensorFusion::RegisterImuPose(double accelX,double accelY,double accelZ,double gyroX,double gyroY,double gyroZ, double time)
{
    boost::mutex::scoped_lock lock(m_ImuLock);
    //add this to the end of the imu data array
    m_lImuData.push_back(ImuData());
    ImuData& data = m_lImuData.back();
    data.m_dAccels = Eigen::Vector3d( accelX,accelY,accelZ);
    data.m_dGyros = Eigen::Vector3d( gyroX,gyroY,gyroZ);
    data.m_dTime = time;

    //advance the latest pose
    if(m_lParams.size() != 0 && m_lImuData.size() > 1){
        //integrate the IMU one step to push the state forward
        std::list<ImuData>::iterator it = m_lImuData.end();
        ImuData& end = *(--it);   //get the last element
        ImuData& start = *(--it);     //get the one before last element

        m_CurrentPose = _IntegrateImuOneStep(m_CurrentPose,start,end,_GetGravityVector(m_CurrentPose.m_dG));
        m_CurrentPose.m_dW = data.m_dGyros;
        m_CurrentPose.m_dTime = time;
    }

}

/////////////////////////////////////////////////////////////////////////////////////////
void SensorFusion::RegisterGlobalPose(Eigen::Vector6d dPose,double time)
{
    //std::cout << "Global pose received at:" << time << "seconds" << std::endl;
    //if there are too many poses, pop one from the beginning
    if(m_lPoses.size() >= m_nFilterSize+1){
        std::list<ImuData>::iterator it = m_lImuData.begin();
        std::list<ImuData>::iterator prev = it;
        //find the first IMU datapoint (interpolate if necessary)
        double tStart = m_lPoses.front().m_dTime;
        while(it != m_lImuData.end() && (*it).m_dTime < tStart) {
            prev = it;
            it++;
        }
        if(prev != it){
            m_lImuData.erase(m_lImuData.begin(),prev);
        }

        //delete all imu poses before this pose
        m_lPoses.pop_front();
    }

    //add one to the new poses array
    m_lPoses.push_back(PoseData());
    PoseData& data = m_lPoses.back();
    data.m_dPose = dPose;
    data.m_dTime = time;

    if(m_lPoses.size() == 1){
        //this was the first pose we pushed back. initialize the current pose
        m_CurrentPose.m_dPose = data.m_dPose;
        m_CurrentPose.m_dTime = data.m_dTime;
        m_CurrentPose.m_dG = Eigen::Vector2d::Zero();
        m_CurrentPose.m_dW = Eigen::Vector3d::Zero();
    }

    //finds the first pose based on the filter size
    int firstPoseIndex = std::max((int)(m_lPoses.size() - m_nFilterSize),0);
    int filterSize = m_lPoses.size() - firstPoseIndex;
    while(m_lParams.size() >= m_nFilterSize+1){
        //delete a parameter from the beginning
        m_lParams.pop_front();
    }

    //add a new parameter for this pose at the end
    //but initialize it to the IMU integrated position
    {
        boost::mutex::scoped_lock lock(m_ImuLock);
        m_CurrentPose.m_dTime = time;
        m_lParams.push_back(m_CurrentPose);
    }

    double norm = DBL_MAX;
    norm = _OptimizePoses();
    //norm = _OptimizePoses();

    //set the current pose as the last parameter
    //if(norm < 1.0){
    {
        boost::mutex::scoped_lock lock(m_ImuLock);
        m_CurrentPose = m_lParams.back();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void SensorFusion::ResetCurrentPose(Eigen::Vector6d pose, const Eigen::Vector3d initV, const Eigen::Vector2d initG)
{
    m_CurrentPose.m_dPose = pose;
    m_CurrentPose.m_dV = initV;
    m_CurrentPose.m_dG = initG;

    //clear all the array
    m_lImuData.clear();
    m_lParams.clear();
    m_lPoses.clear();
}

/////////////////////////////////////////////////////////////////////////////////////////
double SensorFusion::_OptimizePoses()
{
    std::list<PoseData>::iterator currentPose = m_lPoses.begin();
    std::list<PoseParameter>::iterator currentParam = m_lParams.begin();
    int numPoses = m_lParams.size() -1; //we can't move the first pose

    //exit if we don't have enough poses for optimization
    if(numPoses == 0){
        return 0;
    }



    //we need to construct a jacobian
    Eigen::MatrixXd J(numPoses*12,numPoses*6+INITIAL_VEL_TERMS+INITIAL_ACCEL_TERMS);   //6 extra params (3 initial velocity, 3 initial
    J.setZero();
    Eigen::VectorXd e(numPoses*12);  //error vector
    e.setZero();
    //create a weight matrix for the robust norm
    Eigen::SparseMatrix<double> W(e.rows(),e.rows());

    //go through the poses`
    PoseParameter& startParam = m_lParams.front();
    PoseParameter endImuPose = startParam;
    int paramIndex = 0;
    currentParam++;
    currentPose++;    //advance to the first point (the starting point is not optimized)
    while(currentPose !=  m_lPoses.end()){
        double dt = (*currentPose).m_dTime - m_lPoses.front().m_dTime;

        //integrate IMU to this position
        unsigned int numPoses;
        endImuPose = _IntegrateImu(endImuPose,endImuPose.m_dTime,(*currentPose).m_dTime, numPoses);
        endImuPose.m_dTime = (*currentPose).m_dTime;

        Eigen::Matrix4d Tx = mvl::Cart2T((*currentParam).m_dPose);
        Eigen::Matrix4d Ty = Cart2Tinv(endImuPose.m_dPose);
        Eigen::Matrix4d T1 = Tx*Ty;
        e.block<6,1>(paramIndex*12,0) = mvl::T2Cart(T1); //the IMU residual in the error vector

        Eigen::Matrix4d Tz = Cart2Tinv((*currentPose).m_dPose);
        Eigen::Matrix4d T2 = Tx*Tz;
        e.block<6,1>(paramIndex*12 +6,0) = mvl::T2Cart(T2); //the global residual in the error vector

        //create the jacobian of the IMU integration function
        double sp = sin((*currentParam).m_dG[0]);
        double cp = cos((*currentParam).m_dG[0]);
        double sq = sin((*currentParam).m_dG[1]);
        double cq = cos((*currentParam).m_dG[1]);

        Eigen::SparseMatrix<double> dImuInt(6,INITIAL_VEL_TERMS+INITIAL_ACCEL_TERMS);
        dImuInt.coeffRef(0,0) = dt;     dImuInt.coeffRef(0,3) = IMU_GRAVITY_CONST*-sp*sq*0.5*powi(dt,2);      dImuInt.coeffRef(0,4) = IMU_GRAVITY_CONST*cp*cq*0.5*powi(dt,2); //dImuInt.coeffRef(0,3) = -0.5*powi(dt,2);
        dImuInt.coeffRef(1,1) = dt;     dImuInt.coeffRef(1,3) = IMU_GRAVITY_CONST*-cp*0.5*powi(dt,2);
        dImuInt.coeffRef(2,2) = dt;     dImuInt.coeffRef(2,3) = IMU_GRAVITY_CONST*-sp*cq*0.5*powi(dt,2);      dImuInt.coeffRef(2,4) = IMU_GRAVITY_CONST*cp*-sq*0.5*powi(dt,2);

        Eigen::MatrixXd JsVec(16,6);
        ///-------- initial v and g terms on error to IMU
        Eigen::MatrixXdRowMajAlignedVec JsTx = dCart2Tinv(endImuPose.m_dPose);
        for(int jj = 0 ; jj < JsTx.size() ; jj++){
            JsTx[jj] = Tx*JsTx[jj];
            //spaggetify the 4x4 matrix and put it in the correct column (this needs to be done in ROW MAJOR)
            JsTx[jj].resize(16,1);
            JsVec.col(jj) = JsTx[jj];
        }

        JsVec *= dImuInt;  //contribution of the IMU function
        J.block<6,INITIAL_VEL_TERMS+INITIAL_ACCEL_TERMS>(paramIndex*12,0) = dT2Cart(T1)*JsVec; //effect of vInit and gInit on the first error term

        ///-------- parameter terms on error to IMU
        JsVec = Eigen::MatrixXd(16,6);
        Eigen::MatrixXdRowMajAlignedVec JsTy = dCart2T((*currentParam).m_dPose);
        Eigen::MatrixXdRowMajAlignedVec JsTz = JsTy;
        for(int jj = 0 ; jj < JsTy.size() ; jj++){
            JsTy[jj] = JsTy[jj]*Ty;
            //spaggetify the 4x4 matrix and put it in the correct column (this needs to be done in ROW MAJOR)
            JsTy[jj].resize(16,1);
            JsVec.col(jj) = JsTy[jj];
        }
        //effect of parameter terms on the IMU error
        J.block<6,6>    (paramIndex*12,(paramIndex*6) + INITIAL_VEL_TERMS+INITIAL_ACCEL_TERMS) = dT2Cart(T1)*JsVec;



        ///-------- parameter terms on error to Global
        JsVec = Eigen::MatrixXd(16,6);
        for(int jj = 0 ; jj < JsTz.size() ; jj++){
            JsTz[jj] = JsTz[jj]*Tz;
            //spaggetify the 4x4 matrix and put it in the correct column (this needs to be done in ROW MAJOR)
            JsTz[jj].resize(16,1);
            JsVec.col(jj) = JsTz[jj];
        }
        J.block<6,6>(paramIndex*12+6,(paramIndex*6) + INITIAL_VEL_TERMS+INITIAL_ACCEL_TERMS) = dT2Cart(T2)*JsVec; //affect of vInit and gInit on the first error term

        //weight the position error more than the rotation error
        for(int ww = 0; ww < 3 ; ww++){
            W.coeffRef(paramIndex*12 + ww,paramIndex*12 + ww) = 5;    //IMU error XYZ
            W.coeffRef(paramIndex*12 + ww + 3,paramIndex*12 + ww + 3) = 1; //IMU error pqr
            W.coeffRef(paramIndex*12 + ww + 6,paramIndex*12 + ww + 6) = 5; //global error XYZ
            W.coeffRef(paramIndex*12 + ww + 9,paramIndex*12 + ww + 9) = 1; //global error pqr
        }

        paramIndex++;
        currentParam++;
        currentPose++;    //advance to the first point (the starting point is not optimized)
    }

    Eigen::IOFormat CleanFmt(2, 0, ", ", "\n", "[", "]");
    //std::cout << "Error vec:" << e.transpose().format(CleanFmt) << std::endl;

    //std::cout << "J: " << J.format(CleanFmt) << std::endl;


//    for(int ii = 0 ; ii < e.rows() ; ii++){
//        W.coeffRef(ii,ii) = 1.0/fabs(e[ii]);
//    }

    //now that we have the jacobian, solve for the changes in the parameters
    Eigen::VectorXd deltas = J.transpose()*W*e;
    (J.transpose()*W*J).llt().solveInPlace(deltas);

    //std::cout << "deltas: " << deltas.transpose().format(CleanFmt) << std::endl;

    //add the deltas to the initial gravity and velocity
    startParam.m_dV -= deltas.head(INITIAL_VEL_TERMS);
    startParam.m_dG -= deltas.block<INITIAL_ACCEL_TERMS,1>(INITIAL_VEL_TERMS,0) * 0.01 ;

    Eigen::Vector3d g = _GetGravityVector(startParam.m_dG);

    //std::cout << "initV : " << startParam.m_dV.transpose().format(CleanFmt) << " initG : " << g.transpose().format(CleanFmt) << std::endl;

    //add the deltas to all the parameters,
    //and calculate the speed and gravity for each one (gravity is constant and
    //the speed is calculated by integration)
    paramIndex = 0;
    currentParam = m_lParams.begin();
    currentParam++;
    endImuPose = startParam;
    while(currentParam != m_lParams.end()){
        unsigned int numPoses;
        endImuPose = _IntegrateImu(endImuPose,endImuPose.m_dTime,(*currentParam).m_dTime, numPoses);
        (*currentParam).m_dPose -= deltas.block<6,1>(paramIndex*6 + INITIAL_VEL_TERMS+INITIAL_ACCEL_TERMS,0);
        (*currentParam).m_dV = endImuPose.m_dV;
        (*currentParam).m_dG = endImuPose.m_dG;
        endImuPose.m_dTime = (*currentParam).m_dTime;

        //std::cout << "parameter " << paramIndex << " = " << (*currentParam).m_dPose.transpose().format(CleanFmt) << " V=" <<
        //             (*currentParam).m_dV.transpose().format(CleanFmt) << " G=" <<
        //             (*currentParam).m_dG.transpose().format(CleanFmt) << " using " << numPoses << " IMU poses " <<  std::endl;

        paramIndex++;
        currentParam++;
    }

    double norm = deltas.norm();
    //std::cout << "============= with norm " << norm << "=======" <<  std::endl;

    return norm;
}

/////////////////////////////////////////////////////////////////////////////////////////
PoseParameter SensorFusion::_IntegrateImu(const PoseParameter& startingPose, double tStart, double tEnd, unsigned int& numPosesOut)
{
    boost::mutex::scoped_lock lock(m_ImuLock);
    PoseParameter startPose = startingPose;
    //go through the IMU values and integrate along
    std::list<ImuData>::iterator it = m_lImuData.begin();
    std::list<ImuData>::iterator prev = it;
    std::list<ImuData>::iterator next = it;
    numPosesOut = 0;

    //calculate gravity
    //calculate the full gravity vector
    Eigen::Vector3d g = _GetGravityVector(startingPose.m_dG);

    //find the first IMU datapoint (interpolate if necessary)
    while(it != m_lImuData.end() && (*it).m_dTime < tStart) {
        prev = it;
        it++;
    }

    //if we have reached the end m then there is nowhere to go
    if(it == m_lImuData.end()){
        return startPose;
    }

    ImuData interpolatedData = *it;
    if(prev != it){
        //then we have to interpolate
        double alpha = (tStart - (*prev).m_dTime)  / ((*it).m_dTime - (*prev).m_dTime);
        interpolatedData.m_dAccels = (*it).m_dAccels*alpha + (*prev).m_dAccels*(1-alpha);
        interpolatedData.m_dGyros = (*it).m_dGyros*alpha + (*prev).m_dGyros*(1-alpha);
        interpolatedData.m_dTime = tStart;
        numPosesOut++;
        startPose = _IntegrateImuOneStep(startPose,interpolatedData,(*it),g);
    }



    prev = it;
    it++;
    //and now integrate to the last timestep
    while(it != m_lImuData.end() && (*it).m_dTime < tEnd) {
        startPose = _IntegrateImuOneStep(startPose,(*prev),(*it),g);
        prev = it;
        it++;
        numPosesOut++;
    }

    //if we have reached the end m then there is nowhere to go
    if(it == m_lImuData.end()){
        return startPose;
    }

    //then we will have to interpolate the last step
    interpolatedData = *it;
    if(prev != it){
        //then we have to interpolate
        double alpha = (tEnd - (*prev).m_dTime)  / ((*it).m_dTime - (*prev).m_dTime);
        interpolatedData.m_dAccels = (*it).m_dAccels*alpha + (*prev).m_dAccels*(1-alpha);
        interpolatedData.m_dGyros = (*it).m_dGyros*alpha + (*prev).m_dGyros*(1-alpha);
        interpolatedData.m_dTime = tEnd;
        numPosesOut++;
        startPose = _IntegrateImuOneStep(startPose,(*prev),interpolatedData,g);
    }
    return startPose;
}

/////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd SensorFusion::_GetPoseDerivative(Eigen::VectorXd dState, Eigen::Vector3d dG, const ImuData& zStart, const ImuData& zEnd, const double dt)
{
    double sp = sin(dState[3]); // roll phi
    double cp = cos(dState[3]); // roll
    double cq = cos(dState[4]); // pitch
    double tq = tan(dState[4]); // pitch
    double cr = cos(dState[5]); // yaw
    double tr = tan(dState[5]); // yaw

    //matrix from inertial to world coordinates
    Eigen::Matrix3d Rwi = mvl::Cart2R(dState.block<3,1>(3,0));

    //conversion from body rates to angular rates
    Eigen::Matrix3d br2arT;
    br2arT <<   1,   sp*tq,   cp*tq,
                0,      cp,     -sp,
                0,   sp/cq,   cp/cq;

    double alpha = (zEnd.m_dTime - (zStart.m_dTime+dt))/(zEnd.m_dTime - zStart.m_dTime);
    Eigen::Vector3d zw = zStart.m_dGyros*alpha + zEnd.m_dGyros*(1-alpha);
    Eigen::Vector3d za = zStart.m_dAccels*alpha + zEnd.m_dAccels*(1-alpha);


    Eigen::VectorXd deriv(9,1);
    Eigen::Vector3d zw_world = br2arT * zw;
    //derivative of position is velocity
    deriv.head(3) = dState.block<3,1>(6,0);
    deriv.block<3,1>(3,0) = zw_world;
    //centripetal compensation - need the velocity in body coords
//    Eigen::Vector3d v_b = Rwi.transpose() * dState.tail(3);
//    Eigen::Vector3d a_bc = zw.cross(v_b);


//    Eigen::Vector3d v_w = dState.tail(3);
//    Eigen::Vector3d a_wc2 = zw_world.cross(v_w);
//    Eigen::Vector3d a_g = Rwi*za;
//    if(a_bc.norm() > 5.0){
//        int i = 0;
//    }

    deriv.block<3,1>(6,0) = Rwi*(za) + dG ;
    return deriv;
}

/////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector3d SensorFusion::_GetGravityVector(const Eigen::Vector2d &direction)
{
    double sp = sin(direction[0]);
    double cp = cos(direction[0]);
    double sq = sin(direction[1]);
    double cq = cos(direction[1]);
    Eigen::Vector3d g(cp*sq,-sp,cp*cq);
    g *= IMU_GRAVITY_CONST;
    return g;
}

/////////////////////////////////////////////////////////////////////////////////////////
PoseParameter SensorFusion:: _IntegrateImuOneStep(const PoseParameter& currentPose, const ImuData& zStart, const ImuData &zEnd, const Eigen::Vector3d dG)
{
    //construct the state matrix
    Eigen::VectorXd dState(9,1);
    dState.head(6) = currentPose.m_dPose;
    dState.tail(3) = currentPose.m_dV;
    double h = zEnd.m_dTime - zStart.m_dTime;
    if(h == 0){
        return currentPose;
    }
    Eigen::VectorXd k1 = _GetPoseDerivative(dState,dG,zStart,zEnd,0);
    Eigen::VectorXd k2 = _GetPoseDerivative(dState + 0.5*h*k1,dG,zStart,zEnd,0.5*h);
    Eigen::VectorXd k3 = _GetPoseDerivative(dState + 0.5*h*k2,dG,zStart,zEnd,0.5*h);
    Eigen::VectorXd k4 = _GetPoseDerivative(dState + h*k3,dG,zStart,zEnd,h);
    dState = dState + (1.0/6.0)*h*(k1 + 2*k2 + 2*k3 + k4);


    //and now output the state
    PoseParameter output;
    output.m_dPose = dState.head(6);
    output.m_dV = dState.tail(3);
    output.m_dG = currentPose.m_dG;
    output.m_dW = currentPose.m_dW;
    output.m_dTime = zEnd.m_dTime;
    return output;

//    double sp = sin(currentPose.m_dPose[3]); // roll phi
//    double cp = cos(currentPose.m_dPose[3]); // roll
//    double cq = cos(currentPose.m_dPose[4]); // pitch
//    double tq = tan(currentPose.m_dPose[4]); // pitch
//    double cr = cos(currentPose.m_dPose[5]); // yaw
//    double tr = tan(currentPose.m_dPose[5]); // yaw

//    //matrix from inertial to world coordinates
//    Eigen::Matrix3d Rwi = mvl::Cart2R(currentPose.m_dPose.tail(3));

//    //conversion from body rates to angular rates
//    Eigen::Matrix3d br2arT;
//    br2arT <<   1,   sp*tq,   cp*tq,
//                0,      cp,     -sp,
//                0,   sp/cq,   cp/cq;

//    //we will do RK2 here
//    Eigen::Vector3d eulerRatesTi = br2arT * zStart.m_dGyros;
//    Eigen::Vector3d eulerRatesTf = br2arT * zEnd.m_dGyros;
//    Eigen::Vector3d accelTi = Rwi * zStart.m_dAccels - currentPose.m_dG;
//    Eigen::Vector3d accelTf = Rwi * zEnd.m_dAccels - currentPose.m_dG;
//    //calculate the dt
//    double dt = zEnd.m_dTime - zStart.m_dTime;
//    PoseParameter param;
//    param.m_dPose = currentPose.m_dPose;
//    param.m_dV = currentPose.m_dV;


//    param.m_dV += ((accelTi+accelTf)/2)*dt;
//    param.m_dV[2] = 0;
//    param.m_dPose.head(3) += param.m_dV*dt;
//    param.m_dPose.tail(3) += ((eulerRatesTi+eulerRatesTf)/2)*dt;
//    param.m_dG = currentPose.m_dG;

}
