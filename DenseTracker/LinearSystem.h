/*
 * File:   LinearSystem.h
 * Author: jmf
 *
 * Created on July 29, 2012, 1:44 PM
 */

#ifndef LINEARSYSTEM_H
#define LINEARSYSTEM_H

#include <SceneGraph/SimCam.h>
#include <boost/thread.hpp>
#include <opencv/cv.h>

class LinearSystem
{
    public:
        LinearSystem();

        virtual ~LinearSystem();

        void Init(
                const Eigen::Matrix3d& K,				   //< Input: K matrix
                const cv::Mat&         CurImg,             // < Input: Current image
                const cv::Mat&         PrevImg,            // < Input: Image in previous time instance
                const cv::Mat&         PrevDepth,          // < Input: Depth map in previous time instance
                bool                   Decimate = false    // < Input: True if decimate
                );
        Eigen::Matrix4d Solve( unsigned int nNumThreads = 8 );
        void ApplyUpdate();
        double Error();

    private:
        static void _BuildSystem(
                LinearSystem*       pLS,
                const unsigned int& StartU,
                const unsigned int& EndU,
                const unsigned int& StartV,
                const unsigned int& EndV
                );
        Eigen::Vector3d _Project(
                const Eigen::Vector3d& P
                );
        Eigen::Vector3d _BackProject(
                const int&    X,
                const int&    Y,
                const double& Depth
                );
        float _Interpolate(
                const float&   X,       // < Input: X coordinate
                const float&   Y,       // < Input: Y coordinate
                const cv::Mat& Image    // < Input: Image
                );

    private:
        cv::Mat                      m_CurImg;       // grayscale image
        cv::Mat                      m_PrevImg;      // grayscale image
        cv::Mat                      m_PrevDepth;    // depth image
        Eigen::Matrix3d              m_Kv;           // K matrix vision frame
        Eigen::Matrix3d              m_Kr;           // K matrix robotics frame
        unsigned int                 m_nImgWidth;
        unsigned int                 m_nImgHeight;
        Eigen::Matrix4d              m_dTrv;         // Previous estimated transform
        Eigen::Matrix4d              m_dX;           // Estimated transform
        std::vector<Eigen::Matrix4d> m_Gen;          // SE3 generators
        Eigen::Matrix<double, 6, 6>  m_LHS;          // Left Hand Side
        Eigen::Matrix<double, 6, 1>  m_RHS;          // Right Hand Side
        double                       m_dError;
        unsigned int                 m_nErrorPts;
        boost::mutex                 m_Mutex;
        boost::thread_group          m_ThreadGrp;
};
#endif   /* LINEARSYSTEM_H */