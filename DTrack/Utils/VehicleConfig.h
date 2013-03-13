#ifndef _VEHICLE_CONFIG_H_
#define _VEHICLE_CONFIG_H_

#include <Eigen/Dense>

class VehicleConfig
{
    public:
    VehicleConfig(){
        m_vIntrinsics.resize(2);
        m_vSensorPoses.resize(2);
    }

        std::vector< Eigen::Matrix3d >   m_vIntrinsics; // K matrices
        std::vector< Eigen::Matrix4d >   m_vSensorPoses;
};

#endif
