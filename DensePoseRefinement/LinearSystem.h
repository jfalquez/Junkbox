/*
 * File:   LinearSystem.h
 * Author: jmf
 *
 * Created on July 29, 2012, 1:44 PM
 */

#ifndef LINEARSYSTEM_H
#define LINEARSYSTEM_H

#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>
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
//        Eigen::Matrix4d Solve( unsigned int nNumThreads = 8 );
		Eigen::Matrix4d Solve( unsigned int nNumThreads, const std::vector< SceneGraph::ImageView* >& vJac );
        void ApplyUpdate();
		void SnapVirtualCam();
        double Error();
		void Crap(
				const Eigen::Matrix<unsigned char, 1, Eigen::Dynamic>& RefImg2);

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
        double _Interpolate(
                const float&                                           X,       // < Input: X coordinate
                const float&                                           Y,       // < Input: Y coordinate
                const Eigen::Matrix<unsigned char, 1, Eigen::Dynamic>& Image    // < Input: Image
                );
        void _FlipDepth(
                Eigen::VectorXf& vDepth                                         // < Input/Output: Depth buffer
                );
        void _FlipImg(
                Eigen::Matrix<unsigned char, 1, Eigen::Dynamic>& vImg           // < Input/Output: Img buffer
                );

    private:
		SceneGraph::GLSimCam*							m_pVirtCam;
        Eigen::Matrix<unsigned char, 1, Eigen::Dynamic> m_vRefImg;       // greyscale image
        Eigen::Matrix<unsigned char, 1, Eigen::Dynamic> m_vRefImg2;       // greyscale image
        Eigen::Matrix<unsigned char, 1, Eigen::Dynamic> m_vVirtImg;      // greyscale image
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
		Eigen::Matrix<double, 1, 6>						m_J;
		Eigen::Matrix<double, 1, 6>						m_Jo;
        double                                          m_dSSD;
        double                                          m_dError;
        unsigned int                                    m_nErrorPts;
        boost::mutex                                    m_Mutex;

		// Jacobian
		Eigen::VectorXf m_vJImgX;
		Eigen::VectorXf m_vJImgY;
		Eigen::VectorXf m_vJImgZ;
		Eigen::VectorXf m_vJImgP;
		Eigen::VectorXf m_vJImgQ;
		Eigen::VectorXf m_vJImgR;

};
#endif   /* LINEARSYSTEM_H */