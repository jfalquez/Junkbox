#ifndef SENSORFUSION_H
#define SENSORFUSION_H

#include <float.h>
#include <boost/thread.hpp>
#include <Mvlpp/SE3.h>
#include "CarPlannerCommon.h"

#define     INITIAL_VEL_TERMS       3
#define     INITIAL_ACCEL_TERMS     2
#define     IMU_GRAVITY_CONST       9.81

typedef std::vector<Eigen::Matrix<double, Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>,Eigen::aligned_allocator<Eigen::Matrix<double, Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > > VectorXdRowMajAlignedVec;

struct ImuData
{
    Eigen::Vector3d m_dAccels;
    Eigen::Vector3d m_dGyros;
    double m_dTime;
};

struct PoseParameter
{
    Eigen::Vector6d m_dPose;
    Eigen::Vector3d m_dV;
    Eigen::Vector3d m_dW;
    Eigen::Vector2d m_dG;
    double m_dTime;
};

struct PoseData
{
    Eigen::Vector6d m_dPose;
    double m_dTime;
};

class SensorFusion
{

public:
    SensorFusion(const int nFilterSize);
    void RegisterImuPose(double accelX, double accelY, double accelZ, double gyroX, double gyroY, double gyroZ, double time);
    void RegisterGlobalPose(Eigen::Vector6d dPose, double time);
    PoseParameter GetCurrentPose() { return m_CurrentPose; }
    PoseData GetLastGlobalPose() { return m_lPoses.back(); }
    void ResetCurrentPose(Eigen::Vector6d pose, const Eigen::Vector3d initV, const Eigen::Vector2d initG);
    Eigen::Vector3d _GetGravityVector(const Eigen::Vector2d &direction);
private:
    PoseParameter _IntegrateImu(const PoseParameter& startingPose, double tStart, double tEnd, unsigned int &numPosesOut);
    PoseParameter _IntegrateImuOneStep(const PoseParameter& currentPose, const ImuData& zStart, const ImuData &zEnd, const Eigen::Vector3d dG);
    Eigen::VectorXd _GetPoseDerivative(Eigen::VectorXd dState, Eigen::Vector3d dG, const ImuData& zStart, const ImuData& zEnd, const double dt);


    double _OptimizePoses();
    std::list<ImuData> m_lImuData;
    std::list<PoseData> m_lPoses;
    std::list<PoseParameter>  m_lParams;

    PoseParameter m_CurrentPose;

    int m_nFilterSize;

    boost::mutex m_PoseLock;
    boost::mutex m_ImuLock;
};

#endif // SENSORFUSION_H
