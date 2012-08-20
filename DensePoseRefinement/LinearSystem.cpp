/*
 * File:   LinearSystem.cpp
 * Author: jmf
 *
 * Created on July 29, 2012, 1:44 PM
 */

#include <sophus/so3.h>
#include <sophus/se3.h>
#include <Mvlpp/Mvl.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include "LinearSystem.h"


using namespace std;
namespace sg =SceneGraph;

LinearSystem::LinearSystem()
{
    // Initialize generators
    m_Gen.resize( 6 );

    // 1st generator
    m_Gen[0] << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    // 2nd generator
    m_Gen[1] << 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;

    // 3rd generator
    m_Gen[2] << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;

    // 4th generator
    m_Gen[3] << 0, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 0;

    // 5th generator
    m_Gen[4] << 0, 0, 1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0;

    // 6th generator
    m_Gen[5] << 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
}

// //////////////////////////////////////////////////////////////////////////////
LinearSystem::~LinearSystem() {}

// //////////////////////////////////////////////////////////////////////////////
void LinearSystem::Init(
        const Eigen::Matrix<unsigned char, 1, Eigen::Dynamic>& RefImg,     // < Input: Reference image
        sg::GLSimCam*                                          VirtCam,    // < Input: Virtual camera handle
        bool                                                   Decimate    // < Input: True if decimate
        )
{
	m_pVirtCam = VirtCam;

	// ESM init
	m_Jo.setZero();
	m_dSSD = 0;


    // this is such a crappy hack given how FBO works as a singleton
    // we cannot create SimCams with different sizes
    if( Decimate == false ) {
        // store parameters
        m_nImgWidth  = VirtCam->ImageWidth();
        m_nImgHeight = VirtCam->ImageHeight();

		// initialize jacobian image containers
		m_vJImgX.resize(m_nImgWidth * m_nImgHeight);
		m_vJImgY.resize(m_nImgWidth * m_nImgHeight);
		m_vJImgZ.resize(m_nImgWidth * m_nImgHeight);
		m_vJImgP.resize(m_nImgWidth * m_nImgHeight);
		m_vJImgQ.resize(m_nImgWidth * m_nImgHeight);
		m_vJImgR.resize(m_nImgWidth * m_nImgHeight);

        // store K matrix in robotics frame
        // permutation matrix
        Eigen::Matrix3d M;

        M << 0, 1, 0,
				0, 0, 1,
				1, 0, 0;

        m_Kv = VirtCam->GetKMatrix();
        m_Kr = m_Kv * M;

        // RefImg is assumed to have top-left origin
        m_vRefImg = RefImg;
		_FlipImg(m_vRefImg);

		// store virtual image and depth map
		SnapVirtualCam();

        // print initial error
        Eigen::Matrix<unsigned char, 1, Eigen::Dynamic> ImgError;

        ImgError = m_vRefImg - m_vVirtImg;

        // initialize estimate
        m_dTrv = Eigen::Matrix4d::Identity();

        // print initial error
        m_nErrorPts = m_nImgHeight * m_nImgWidth;
        m_dError    = ImgError.lpNorm<1>();

        std::cout << "Error is: " << Error() << std::endl;
	}
}

// //////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4d LinearSystem::Solve( unsigned int nNumThreads, const vector< sg::ImageView* >& vJac )
{
	// reset Jacobians
	m_J.setZero();

    // reset LHS + RHS
    m_LHS.setZero();
    m_RHS.setZero();

    // reset error
    m_dError    = 0;
    m_nErrorPts = 0;

	// previous SSD error
	double dError = m_dSSD;
	m_dSSD = 0;

    boost::thread*		pT;
	boost::thread_group ThreadGrp;

	if( nNumThreads == 4 ) {

		// 4 ways
		pT = new boost::thread( _BuildSystem, this, 0, m_nImgWidth, 0, m_nImgHeight / 4 );

		ThreadGrp.add_thread( pT );

		pT = new boost::thread( _BuildSystem, this, 0, m_nImgWidth, m_nImgHeight / 4, m_nImgHeight / 2 );

		ThreadGrp.add_thread( pT );

		pT = new boost::thread( _BuildSystem, this, 0, m_nImgWidth, m_nImgHeight / 2, 3 * m_nImgHeight / 4 );

		ThreadGrp.add_thread( pT );

		_BuildSystem( this, 0, m_nImgWidth, 3 * m_nImgHeight / 4, m_nImgHeight );

	    // wait for all threads to finish
		ThreadGrp.join_all();

	} else if( nNumThreads == 8 ) {

		// 8 ways
		pT = new boost::thread( _BuildSystem, this, 0, m_nImgWidth, 0, m_nImgHeight / 8 );

		ThreadGrp.add_thread( pT );

		pT = new boost::thread( _BuildSystem, this, 0, m_nImgWidth, m_nImgHeight / 8, m_nImgHeight / 4 );

		ThreadGrp.add_thread( pT );

		pT = new boost::thread( _BuildSystem, this, 0, m_nImgWidth, m_nImgHeight / 4, 3 * m_nImgHeight / 8 );

		ThreadGrp.add_thread( pT );

		pT = new boost::thread( _BuildSystem, this, 0, m_nImgWidth, 3 * m_nImgHeight / 8, m_nImgHeight / 2 );

		ThreadGrp.add_thread( pT );

		pT = new boost::thread( _BuildSystem, this, 0, m_nImgWidth, m_nImgHeight / 2, 5 * m_nImgHeight / 8 );

		ThreadGrp.add_thread( pT );

		pT = new boost::thread( _BuildSystem, this, 0, m_nImgWidth, 5 * m_nImgHeight / 8, 3 * m_nImgHeight / 4 );

		ThreadGrp.add_thread( pT );

		pT = new boost::thread( _BuildSystem, this, 0, m_nImgWidth, 3 * m_nImgHeight / 4, 7 * m_nImgHeight / 8 );

		ThreadGrp.add_thread( pT );

		_BuildSystem( this, 0, m_nImgWidth, 7 * m_nImgHeight / 8, m_nImgHeight );

	    // wait for all threads to finish
		ThreadGrp.join_all();
	}
	else {
		// 1 thread or an invalid number was requested
		_BuildSystem( this, 0, m_nImgWidth, 0, m_nImgHeight );
	}

	// update variable
    Eigen::Vector6d Delta;

	// solve via ESM
//	if( m_Jo.norm() != 0 ) {
//		m_J = (m_Jo + m_J) / 2.0;
//	}

//    Delta = m_J.inverse() * -m_dSSD;

	// solve via GN
	Delta = m_LHS.ldlt().solve( m_RHS );

	// set J as previous Jo
	m_Jo = m_J;

	// normalize jacobians
	float Max = 0;
	float Maxi;

	Max = m_vJImgX.maxCoeff();
	Maxi = m_vJImgY.maxCoeff();
	if( Maxi > Max ) {
		Max = Maxi;
	}
	Maxi = m_vJImgZ.maxCoeff();
	if( Maxi > Max ) {
		Max = Maxi;
	}
	Maxi = m_vJImgP.maxCoeff();
	if( Maxi > Max ) {
		Max = Maxi;
	}
	Maxi = m_vJImgQ.maxCoeff();
	if( Maxi > Max ) {
		Max = Maxi;
	}
	Maxi = m_vJImgR.maxCoeff();
	if( Maxi > Max ) {
		Max = Maxi;
	}
//	m_vJImgX = m_vJImgX / Max;
//	m_vJImgY = m_vJImgY / Max;
//	m_vJImgZ = m_vJImgZ / Max;
//	m_vJImgP = m_vJImgP / Max;
//	m_vJImgQ = m_vJImgQ / Max;
//	m_vJImgR = m_vJImgR / Max;
	m_vJImgX = m_vJImgX / m_vJImgX.maxCoeff();
	m_vJImgY = m_vJImgY / m_vJImgY.maxCoeff();
	m_vJImgZ = m_vJImgZ / m_vJImgZ.maxCoeff();
	m_vJImgP = m_vJImgP / m_vJImgP.maxCoeff();
	m_vJImgQ = m_vJImgQ / m_vJImgQ.maxCoeff();
	m_vJImgR = m_vJImgR / m_vJImgR.maxCoeff();
	vJac[0]->SetImage( m_vJImgX.data(), m_nImgWidth, m_nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_FLOAT );
	vJac[1]->SetImage( m_vJImgY.data(), m_nImgWidth, m_nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_FLOAT );
	vJac[2]->SetImage( m_vJImgZ.data(), m_nImgWidth, m_nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_FLOAT );
	vJac[3]->SetImage( m_vJImgP.data(), m_nImgWidth, m_nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_FLOAT );
	vJac[4]->SetImage( m_vJImgQ.data(), m_nImgWidth, m_nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_FLOAT );
	vJac[5]->SetImage( m_vJImgR.data(), m_nImgWidth, m_nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_FLOAT );

    // convert Delta from Lie
    Sophus::SE3 DeltaPose;

    DeltaPose = Sophus::SE3::exp( Delta );

    m_dX      = DeltaPose.matrix();
    return m_dX;
}

// //////////////////////////////////////////////////////////////////////////////
void LinearSystem::ApplyUpdate()
{
    m_dTrv = m_dTrv * mvl::TInv( m_dX );
}

// //////////////////////////////////////////////////////////////////////////////
void LinearSystem::SnapVirtualCam()
{
	// resize vectors
	m_vVirtImg.resize( m_nImgWidth * m_nImgHeight );
	m_vVirtDepth.resize( m_nImgWidth * m_nImgHeight );

	// populate vectors
	m_pVirtCam->CaptureGrey( m_vVirtImg.data() );
	_FlipImg(m_vVirtImg);
	m_pVirtCam->CaptureDepth( m_vVirtDepth.data() );
	_FlipDepth(m_vVirtDepth);
}

// //////////////////////////////////////////////////////////////////////////////
double LinearSystem::Error()
{
    return m_dError / m_nErrorPts;
}

/**************************************************************************************************
 *
 * PRIVATE METHODS
 *
 **************************************************************************************************/
#if 1
void LinearSystem::_BuildSystem(
        LinearSystem*       pLS,
        const unsigned int& StartU,
        const unsigned int& EndU,
        const unsigned int& StartV,
        const unsigned int& EndV
        )
{
    // Jacobian
    Eigen::Matrix<double, 1, 6> J;

    J.setZero();

    Eigen::Matrix<double, 6, 6> LHS;    // Left Hand Side

    LHS.setZero();

    Eigen::Matrix<double, 6, 1> RHS;    // Right Hand Side

    RHS.setZero();

    double       Error    = 0;
    unsigned int ErrorPts = 0;

    for( int ii = StartV; ii < EndV; ii++ ) {
        for( int jj = StartU; jj < EndU; jj++ ) {
            // variables
            Eigen::Vector2d Ur;        // pixel position
            Eigen::Vector3d Pr, Pv;    // 3d point
            Eigen::Vector4d Ph;        // homogenized point

            // check if pixel is contained in our model (i.e. has depth)
            if( pLS->m_vVirtDepth[ii * pLS->m_nImgWidth + jj] == 0 ) {
                continue;
            }

            // --------------------- first term 1x2
            // evaluate 'a' = L[ Trv * Linv( Uv ) ]
            // back project to virtual camera's reference frame
            // this already brings points to robotics reference frame
            Pv = pLS->_BackProject( jj, ii, pLS->m_vVirtDepth[ii * pLS->m_nImgWidth + jj] );

            // convert to homogeneous coordinate
            Ph << Pv, 1;

            // transform point to reference camera's frame
            // Pr = Trv * Pv
            Ph = pLS->m_dTrv * Ph;
            Pr = Ph.head( 3 );

            // project onto reference camera
            Eigen::Vector3d Lr;

            Lr = pLS->_Project( Pr );
            Ur = Lr.head( 2 );
            Ur = Ur / Lr( 2 );

            // check if point falls in camera's field of view
            if( (Ur( 0 ) <= 1) || (Ur( 0 ) >= pLS->m_nImgWidth - 2) || (Ur( 1 ) <= 1)
                    || (Ur( 1 ) >= pLS->m_nImgHeight - 2) ) {
                continue;
            }

            // finite differences
            float                       TopPix   = pLS->_Interpolate( Ur( 0 ), Ur( 1 ) - 1, pLS->m_vRefImg );
            float                       BotPix   = pLS->_Interpolate( Ur( 0 ), Ur( 1 ) + 1, pLS->m_vRefImg );
            float                       LeftPix  = pLS->_Interpolate( Ur( 0 ) - 1, Ur( 1 ), pLS->m_vRefImg );
            float                       RightPix = pLS->_Interpolate( Ur( 0 ) + 1, Ur( 1 ), pLS->m_vRefImg );
            Eigen::Matrix<double, 1, 2> Term1;

            Term1( 0 ) = (RightPix - LeftPix) / 2.0;
            Term1( 1 ) = (BotPix - TopPix) / 2.0;

            // --------------------- second term 2x3
            // evaluate 'b' = Trv * Linv( Uv )
            // this was already calculated for Term1
            // fill matrix
            // 1/c      0       -a/c^2
            // 0       1/c     -b/c^2
            Eigen::Matrix<double, 2, 3> Term2;
            double                      PowC = Lr( 2 ) * Lr( 2 );

            Term2( 0, 0 ) = 1.0 / Lr( 2 );
            Term2( 0, 1 ) = 0;
            Term2( 0, 2 ) = -(Lr( 0 )) / PowC;
            Term2( 1, 0 ) = 0;
            Term2( 1, 1 ) = 1.0 / Lr( 2 );
            Term2( 1, 2 ) = -(Lr( 1 )) / PowC;
            Term2         = Term2 * pLS->m_Kr;

            // --------------------- third term 3x1
            // we need Pv in homogenous coordinates
            Ph << Pv, 1;

            Eigen::Vector4d Term3i;

            // last row of Term3 is truncated since it is always 0
            Eigen::Vector3d Term3;

            // fill Jacobian with T generators
            Term3i    = pLS->m_dTrv * pLS->m_Gen[0] * Ph;
            Term3     = Term3i.head( 3 );
            J( 0, 0 ) = Term1 * Term2 * Term3;
			pLS->m_vJImgX[ii * pLS->m_nImgWidth + jj] = J(0, 0);
            Term3i    = pLS->m_dTrv * pLS->m_Gen[1] * Ph;
            Term3     = Term3i.head( 3 );
            J( 0, 1 ) = Term1 * Term2 * Term3;
			pLS->m_vJImgY[ii * pLS->m_nImgWidth + jj] = J(0, 1);
            Term3i    = pLS->m_dTrv * pLS->m_Gen[2] * Ph;
            Term3     = Term3i.head( 3 );
            J( 0, 2 ) = Term1 * Term2 * Term3;
			pLS->m_vJImgZ[ii * pLS->m_nImgWidth + jj] = J(0, 2);
            Term3i    = pLS->m_dTrv * pLS->m_Gen[3] * Ph;
            Term3     = Term3i.head( 3 );
            J( 0, 3 ) = Term1 * Term2 * Term3;
			pLS->m_vJImgP[ii * pLS->m_nImgWidth + jj] = J(0, 3);
            Term3i    = pLS->m_dTrv * pLS->m_Gen[4] * Ph;
            Term3     = Term3i.head( 3 );
            J( 0, 4 ) = Term1 * Term2 * Term3;
			pLS->m_vJImgQ[ii * pLS->m_nImgWidth + jj] = J(0, 4);
            Term3i    = pLS->m_dTrv * pLS->m_Gen[5] * Ph;
            Term3     = Term3i.head( 3 );
            J( 0, 5 ) = Term1 * Term2 * Term3;
			pLS->m_vJImgR[ii * pLS->m_nImgWidth + jj] = J(0, 5);

            // estimate LHS (Hessian)
            // LHS = Hessian = Jt * J
            LHS += J.transpose() * J;

            // std::cout << "J is: " << std::endl << J << std::endl;
            // estimate RHS (error)
            // RHS = Jt * e
            RHS += J.transpose()
                   * (pLS->_Interpolate( Ur( 0 ), Ur( 1 ), pLS->m_vRefImg )
                      - pLS->m_vVirtImg[ii * pLS->m_nImgWidth + jj]);

            // calculate normalized error
            Error += fabs( pLS->_Interpolate( Ur( 0 ), Ur( 1 ), pLS->m_vRefImg )
                           - pLS->m_vVirtImg[ii * pLS->m_nImgWidth + jj] );

            ErrorPts++;
        }
    }

    // update global LHS and RHS
    // ---------- start contention zone
    pLS->m_Mutex.lock();

    pLS->m_LHS       += LHS;
    pLS->m_RHS       += RHS;
    pLS->m_dError    += Error;
    pLS->m_nErrorPts += ErrorPts;

    pLS->m_Mutex.unlock();

    // ---------- end contention zone
}
#else
void LinearSystem::_BuildSystem(
        LinearSystem*       pLS,
        const unsigned int& StartU,
        const unsigned int& EndU,
        const unsigned int& StartV,
        const unsigned int& EndV
        )
{
    // Jacobian
    Eigen::Matrix<double, 1, 6> BigJ;
    Eigen::Matrix<double, 1, 6> J;

    BigJ.setZero();
    J.setZero();

	// Errors
	double		 e;
	double		 SSD = 0;
    double       Error    = 0;
    unsigned int ErrorPts = 0;

    for( int ii = StartV; ii < EndV; ii++ ) {
        for( int jj = StartU; jj < EndU; jj++ ) {
            // variables
            Eigen::Vector2d Ur;        // pixel position
            Eigen::Vector3d Pr, Pv;    // 3d point
            Eigen::Vector4d Ph;        // homogenized point

            // check if pixel is contained in our model (i.e. has depth)
            if( pLS->m_vVirtDepth[ii * pLS->m_nImgWidth + jj] == 0 ) {
                continue;
            }

            // --------------------- first term 1x2
            // evaluate 'a' = L[ Trv * Linv( Uv ) ]
            // back project to virtual camera's reference frame
            // this already brings points to robotics reference frame
            Pv = pLS->_BackProject( jj, ii, pLS->m_vVirtDepth[ii * pLS->m_nImgWidth + jj] );

            // convert to homogeneous coordinate
            Ph << Pv, 1;

            // transform point to reference camera's frame
            // Pr = Trv * Pv
            Ph = pLS->m_dTrv * Ph;
            Pr = Ph.head( 3 );

            // project onto reference camera
            Eigen::Vector3d Lr;

            Lr = pLS->_Project( Pr );
            Ur = Lr.head( 2 );
            Ur = Ur / Lr( 2 );

            // check if point falls in camera's field of view
            if( (Ur( 0 ) <= 1) || (Ur( 0 ) >= pLS->m_nImgWidth - 2) || (Ur( 1 ) <= 1)
                    || (Ur( 1 ) >= pLS->m_nImgHeight - 2) ) {
                continue;
            }

            // finite differences
            float                       TopPix   = pLS->_Interpolate( Ur( 0 ), Ur( 1 ) - 1, pLS->m_vRefImg );
            float                       BotPix   = pLS->_Interpolate( Ur( 0 ), Ur( 1 ) + 1, pLS->m_vRefImg );
            float                       LeftPix  = pLS->_Interpolate( Ur( 0 ) - 1, Ur( 1 ), pLS->m_vRefImg );
            float                       RightPix = pLS->_Interpolate( Ur( 0 ) + 1, Ur( 1 ), pLS->m_vRefImg );
            Eigen::Matrix<double, 1, 2> Term1;

            Term1( 0 ) = (RightPix - LeftPix) / 2.0;
            Term1( 1 ) = (BotPix - TopPix) / 2.0;

            // --------------------- second term 2x3
            // evaluate 'b' = Trv * Linv( Uv )
            // this was already calculated for Term1
            // fill matrix
            // 1/c      0       -a/c^2
            // 0       1/c     -b/c^2
            Eigen::Matrix<double, 2, 3> Term2;
            double                      PowC = Lr( 2 ) * Lr( 2 );

            Term2( 0, 0 ) = 1.0 / Lr( 2 );
            Term2( 0, 1 ) = 0;
            Term2( 0, 2 ) = -(Lr( 0 )) / PowC;
            Term2( 1, 0 ) = 0;
            Term2( 1, 1 ) = 1.0 / Lr( 2 );
            Term2( 1, 2 ) = -(Lr( 1 )) / PowC;
            Term2         = Term2 * pLS->m_Kr;

            // --------------------- third term 3x1
            // we need Pv in homogenous coordinates
            Ph << Pv, 1;

            Eigen::Vector4d Term3i;

            // last row of Term3 is truncated since it is always 0
            Eigen::Vector3d Term3;

            // fill Jacobian with T generators
            Term3i    = pLS->m_dTrv * pLS->m_Gen[0] * Ph;
            Term3     = Term3i.head( 3 );
            J( 0, 0 ) = Term1 * Term2 * Term3;
			pLS->m_vJImgX[(StartV + ii) * pLS->m_nImgWidth + (StartU) + jj] = J(0, 0);
            Term3i    = pLS->m_dTrv * pLS->m_Gen[1] * Ph;
            Term3     = Term3i.head( 3 );
            J( 0, 1 ) = Term1 * Term2 * Term3;
			pLS->m_vJImgY[(StartV + ii) * pLS->m_nImgWidth + (StartU) + jj] = J(0, 1);
            Term3i    = pLS->m_dTrv * pLS->m_Gen[2] * Ph;
            Term3     = Term3i.head( 3 );
            J( 0, 2 ) = Term1 * Term2 * Term3;
			pLS->m_vJImgZ[(StartV + ii) * pLS->m_nImgWidth + (StartU) + jj] = J(0, 2);
            Term3i    = pLS->m_dTrv * pLS->m_Gen[3] * Ph;
            Term3     = Term3i.head( 3 );
            J( 0, 3 ) = Term1 * Term2 * Term3;
			pLS->m_vJImgP[(StartV + ii) * pLS->m_nImgWidth + (StartU) + jj] = J(0, 3);
            Term3i    = pLS->m_dTrv * pLS->m_Gen[4] * Ph;
            Term3     = Term3i.head( 3 );
            J( 0, 4 ) = Term1 * Term2 * Term3;
			pLS->m_vJImgQ[(StartV + ii) * pLS->m_nImgWidth + (StartU) + jj] = J(0, 4);
            Term3i    = pLS->m_dTrv * pLS->m_Gen[5] * Ph;
            Term3     = Term3i.head( 3 );
            J( 0, 5 ) = Term1 * Term2 * Term3;
			pLS->m_vJImgR[(StartV + ii) * pLS->m_nImgWidth + (StartU) + jj] = J(0, 5);

            // accumulate Jacobian
            BigJ += J;

			// accumulate SSD error
			e = pLS->_Interpolate( Ur( 0 ), Ur( 1 ), pLS->m_vRefImg )
					- pLS->m_vVirtImg[ii * pLS->m_nImgWidth + jj];

			SSD += e * e;

            // calculate normalized error
            Error += fabs( pLS->_Interpolate( Ur( 0 ), Ur( 1 ), pLS->m_vRefImg )
                           - pLS->m_vVirtImg[ii * pLS->m_nImgWidth + jj] );

            ErrorPts++;
        }
    }

    // update global LHS and RHS
    // ---------- start contention zone
    pLS->m_Mutex.lock();

    pLS->m_J         += BigJ;
	pLS->m_dSSD		 += SSD;
    pLS->m_dError    += Error;
    pLS->m_nErrorPts += ErrorPts;

    pLS->m_Mutex.unlock();

    // ---------- end contention zone
}
#endif


// //////////////////////////////////////////////////////////////////////////////
inline Eigen::Vector3d LinearSystem::_Project(
        const Eigen::Vector3d& P
        )
{
    Eigen::Vector3d T = P;

//    T = m_Kv * T;
    T = m_Kr * T;

    if( T( 2 ) == 0 ) {
        std::cout << "Oops! I just saved you from making a division by zero!" << std::endl;

        exit( 0 );
    }

    return T;
}

// //////////////////////////////////////////////////////////////////////////////
inline Eigen::Vector3d LinearSystem::_BackProject(
        const int&    X,
        const int&    Y,
        const double& Depth
        )
{
    // get camera intrinsics
    double          cx = m_Kv( 0, 2 );
    double          cy = m_Kv( 1, 2 );
    double          fx = m_Kv( 0, 0 );
    double          fy = m_Kv( 1, 1 );
    Eigen::Vector3d P;

//    P( 0 ) = Depth * ((X - cx) / fx);
//    P( 1 ) = Depth * ((Y - cy) / fy);
//    P( 2 ) = Depth;

	P( 1 ) = Depth * ((X - cx) / fx);
    P( 2 ) = Depth * ((Y - cy) / fy);
    P( 0 ) = Depth;

    return P;
}

// //////////////////////////////////////////////////////////////////////////////
inline float LinearSystem::_Interpolate(
        const float&                                           X,       // < Input: X coordinate
        const float&                                           Y,       // < Input: Y coordinate
        const Eigen::Matrix<unsigned char, 1, Eigen::Dynamic>& Image    // < Input: Image
        )
{
    int          xt  = (int)X;    /* top-left corner */
    int          yt  = (int)Y;
    float        ax  = X - xt;
    float        ay  = Y - yt;
    unsigned int idx = (m_nImgWidth * yt) + xt;

    if( (xt >= 0) && (yt >= 0) && (xt <= m_nImgWidth - 2) && (yt <= m_nImgHeight - 2) ) {
        return((1 - ax) * (1 - ay) * Image[idx] + (ax)*(1 - ay) * Image[idx + 1]
               + (1 - ax) * (ay)*Image[idx + m_nImgWidth] + (ax)*(ay)*Image[idx + m_nImgWidth + 1]);
    }

    return 0;
}

// //////////////////////////////////////////////////////////////////////////////
inline void LinearSystem::_FlipDepth(
        Eigen::VectorXf& vDepth    // < Input/Output: Depth buffer
        )
{
    Eigen::VectorXf tmp;

    tmp = vDepth;

    unsigned int Idx;

    for( int ii = 0; ii < m_nImgHeight; ii++ ) {
        for( int jj = 0; jj < m_nImgWidth; jj++ ) {
            Idx                           = (m_nImgHeight - ii - 1) * m_nImgWidth + jj;
            vDepth[ii * m_nImgWidth + jj] = tmp[Idx];
        }
    }
}

// //////////////////////////////////////////////////////////////////////////////
inline void LinearSystem::_FlipImg(
        Eigen::Matrix<unsigned char, 1, Eigen::Dynamic>& vImg    // < Input/Output: Img buffer
        )
{
    Eigen::Matrix<unsigned char, 1, Eigen::Dynamic> tmp;

    tmp = vImg;

    unsigned int Idx;

    for( int ii = 0; ii < m_nImgHeight; ii++ ) {
        for( int jj = 0; jj < m_nImgWidth; jj++ ) {
            Idx                         = (m_nImgHeight - ii - 1) * m_nImgWidth + jj;
            vImg[ii * m_nImgWidth + jj] = tmp[Idx];
        }
    }
}