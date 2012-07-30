/*
 * File:   LinearSystem.h
 * Author: jmf
 *
 * Created on July 29, 2012, 1:44 PM
 */

#ifndef LINEARSYSTEM_H
#define LINEARSYSTEM_H

#include <SimpleGui/Gui.h>

class LinearSystem
{
    public:
        LinearSystem();

        virtual ~LinearSystem();

        void Init(
                GLSimCam* RefCam,
                GLSimCam* VirtCam
                );
        Eigen::Matrix4d Solve();
        void ApplyUpdate();
        double Error();

    private:
        void _BuildSystem();
        Eigen::Vector3d _Project(
                const Eigen::Vector3d& P
                );
        Eigen::Vector3d _BackProject(
                const int&    X,
                const int&    Y,
                const double& Depth
                );
        float _Interpolate(
                const float&           X,                 // < Input: X coordinate
                const float&           Y,                 // < Input: Y coordinate
                const Eigen::VectorXf& Image              // < Input: Image
                );
        void _RGB2Gray(
                const std::vector<unsigned char>& RGB,    // < Input: RGB image
                Eigen::VectorXf&                  Gray    // < Output: Grayscale image
                );
        void _FlipDepth(
                Eigen::VectorXf& vDepth                   // < Input/Output: Depth buffer
                );

    private:
        Eigen::VectorXf              m_vRefImg;       // grayscale reference image
        Eigen::VectorXf              m_vVirtImg;      // grayscale virtual image
        Eigen::VectorXf              m_vVirtDepth;    // depth image from virtual camera
        Eigen::Matrix3d              m_Kv;            // vision frame K matrix
        Eigen::Matrix3d              m_Kr;            // robotics frame K matrix
        unsigned int                 m_nImgWidth;
        unsigned int                 m_nImgHeight;
        Eigen::Matrix4d              m_dTrv;          // Previous estimated transform
        Eigen::Matrix4d              m_dX;            // Estimated transform
        std::vector<Eigen::Matrix4d> m_Gen;           // SE3 generators
        Eigen::Matrix<double, 6, 6>  m_LHS;           // Left Hand Side
        Eigen::Matrix<double, 6, 1>  m_RHS;           // Right Hand Side
        double                       m_dError;
        unsigned int                 m_nErrorPts;
        boost::mutex                 m_Mutex;
};
#endif   /* LINEARSYSTEM_H */