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

class LinearSystem
{
    public:
        LinearSystem();

        virtual ~LinearSystem();

		void Init(
				const Eigen::Matrix<unsigned char, 1, Eigen::Dynamic>& RefImg,	// < Input: Reference image
				SceneGraph::GLSimCam* VirtCam,												// < Input: Virtual camera handle
				bool Decimate = false											// < Input: True if decimate
				);
        Eigen::Matrix4d Solve( unsigned int nNumThreads = 8 );
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
        Eigen::Vector3d _Project(
                const Eigen::Vector3d& P
                );
        Eigen::Vector3d _BackProject(
                const int&    X,
                const int&    Y,
                const double& Depth
                );
        float _Interpolate(
                const float&                                           X,       // < Input: X coordinate
                const float&                                           Y,       // < Input: Y coordinate
                const Eigen::Matrix<unsigned char, 1, Eigen::Dynamic>& Image    // < Input: Image
                );

    private:
		SceneGraph::GLSimCam*							m_pVirtCam;
        Eigen::Matrix<unsigned char, 1, Eigen::Dynamic> m_vRefImg;       // grayscale image
        Eigen::Matrix<unsigned char, 1, Eigen::Dynamic> m_vVirtImg;      // grayscale image
        Eigen::VectorXf                                 m_vVirtDepth;    // depth image from virtual camera
        Eigen::Matrix3d                                 m_Kv;            // vision frame K matrix
        Eigen::Matrix3d                                 m_Kr;            // robotics frame K matrix
        unsigned int                                    m_nImgWidth;
        unsigned int                                    m_nImgHeight;
        Eigen::Matrix4d                                 m_dTrv;          // Previous estimated transform
        Eigen::Matrix4d                                 m_dX;            // Estimated transform
        std::vector<Eigen::Matrix4d>                    m_Gen;           // SE3 generators
        Eigen::Matrix<double, 6, 6>                     m_LHS;           // Left Hand Side
        Eigen::Matrix<double, 6, 1>                     m_RHS;           // Right Hand Side
        double                                          m_dError;
        unsigned int                                    m_nErrorPts;
        boost::mutex                                    m_Mutex;
        boost::thread_group                             m_ThreadGrp;
};
#endif   /* LINEARSYSTEM_H */