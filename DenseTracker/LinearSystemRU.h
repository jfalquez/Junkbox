/*
* File:   LinearSystem.h
* Author: jmf
*
* Created on July 29, 2012, 1:44 PM
 */

#ifndef LINEARSYSTEM_RU_H
#define LINEARSYSTEM_RU_H

#include <SceneGraph/SimCam.h>
#include <opencv.hpp>
#include <boost/thread.hpp>

class LinearSystem
{
    public:
        LinearSystem();

        virtual ~LinearSystem();

        void Init(
                const cv::Mat&        RefImg,             // < Input: Reference image
                SceneGraph::GLSimCam* VirtCam,            // < Input: Virtual camera handle
                bool                  Decimate = false    // < Input: True if decimate
                );
        Eigen::Matrix4d Solve(
                unsigned int nNumThreads = 8
                );
        void ApplyUpdate();
        void SnapVirtualCam();
        double Error();

    private:
        static void _BuildSystem(
                LinearSystem*       pLS,
                const unsigned int& StartU,
                const unsigned int& EndU,
                const unsigned int& StartV,
                const unsigned int& EndV
                );
        Eigen::Vector3d _BackProject(
                const int&    X,
                const int&    Y,
                const double& Depth
                );

    private:
        SceneGraph::GLSimCam*          m_pVirtCam;
        cv::Mat                        m_vRefImg;       // grayscale image
        cv::Mat                        m_vFDImgBT;      // image finite differences - Up/Down
        cv::Mat                        m_vFDImgRL;      // image finite differences - Left/Right
        cv::Mat                        m_vVirtImg;      // grayscale image
        cv::Mat                        m_vVirtDepth;    // depth image from virtual camera
        Eigen::Matrix3d                m_Kv;            // vision frame K matrix
        Eigen::Matrix3d                m_Kr;            // robotics frame K matrix
        unsigned int                   m_nImgWidth;
        unsigned int                   m_nImgHeight;
        Eigen::Matrix4d                m_dTrv;          // Previous estimated transform
        Eigen::Matrix4d                m_dX;            // Estimated transform
        std::vector< Eigen::Matrix4d > m_Gen;           // SE3 generators
        Eigen::Matrix< double, 6, 6 >  m_LHS;           // Left Hand Side
        Eigen::Matrix< double, 6, 1 >  m_RHS;           // Right Hand Side
        double                         m_dError;
        unsigned int                   m_nErrorPts;
        boost::mutex                   m_Mutex;
        boost::thread_group            m_ThreadGrp;
};
#endif   /* LINEARSYSTEM_H */