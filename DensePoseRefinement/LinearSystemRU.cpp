/*
 * File:   LinearSystem.cpp
 * Author: jmf
 *
 * Created on July 29, 2012, 1:44 PM
 */

#include "LinearSystemRU.h"
#include <sophus/so3.h>
#include <sophus/se3.h>
#include <Mvlpp/Mvl.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

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

    // this is such a crappy hack given how FBO works as a singleton
    // we cannot create SimCams with different sizes
    if( Decimate == false ) {
        // store parameters
        m_nImgWidth  = VirtCam->ImageWidth();
        m_nImgHeight = VirtCam->ImageHeight();

        // store K matrix in robotics frame
        // permutation matrix
        Eigen::Matrix3d M;

        M << 0, 1, 0, 0, 0, 1, 1, 0, 0;

        m_Kv = VirtCam->GetKMatrix();
        m_Kr = m_Kv * M;

        // RefImg is assumed to have top-left origin
        m_vRefImg = RefImg;

		// pre-calculate finite difference matrix
		unsigned int TopPix;
		unsigned int BotPix;
		unsigned int LeftPix;
		unsigned int RightPix;

		m_vFDImgBT.resize( m_nImgWidth * m_nImgHeight );
		m_vFDImgRL.resize( m_nImgWidth * m_nImgHeight );

		for( int ii = 1; ii < m_nImgHeight-1; ii++ ) {
			for( int jj = 1; jj < m_nImgWidth-1; jj++ ) {
				TopPix = m_vRefImg[ (ii - 1) * m_nImgWidth + jj ];
				BotPix = m_vRefImg[ (ii + 1) * m_nImgWidth + jj ];
				LeftPix = m_vRefImg[ ii * m_nImgWidth + jj - 1];
				RightPix = m_vRefImg[ ii * m_nImgWidth + jj + 1 ];
				m_vFDImgBT[ii * m_nImgWidth + jj] = (BotPix - TopPix) / 2.0;
				m_vFDImgRL[ii * m_nImgWidth + jj] = (RightPix - LeftPix) / 2.0;
			}
		}

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
Eigen::Matrix4d LinearSystem::Solve( unsigned int nNumThreads )
{
    // reset LHS + RHS
    m_LHS.setZero();
    m_RHS.setZero();

    // reset error
    m_dError    = 0;
    m_nErrorPts = 0;

    boost::thread * pT;

	if( nNumThreads == 4 ) {

		// 4 ways
		pT = new boost::thread( _BuildSystem, this, 0, m_nImgWidth, 0, m_nImgHeight / 4 );

		m_ThreadGrp.add_thread( pT );

		pT = new boost::thread( _BuildSystem, this, 0, m_nImgWidth, m_nImgHeight / 4, m_nImgHeight / 2 );

		m_ThreadGrp.add_thread( pT );

		pT = new boost::thread( _BuildSystem, this, 0, m_nImgWidth, m_nImgHeight / 2, 3 * m_nImgHeight / 4 );

		m_ThreadGrp.add_thread( pT );

		_BuildSystem( this, 0, m_nImgWidth, 3 * m_nImgHeight / 4, m_nImgHeight );

	    // wait for all threads to finish
		m_ThreadGrp.join_all();

	} else if( nNumThreads == 8 ) {

		// 8 ways
		pT = new boost::thread( _BuildSystem, this, 0, m_nImgWidth, 0, m_nImgHeight / 8 );

		m_ThreadGrp.add_thread( pT );

		pT = new boost::thread( _BuildSystem, this, 0, m_nImgWidth, m_nImgHeight / 8, m_nImgHeight / 4 );

		m_ThreadGrp.add_thread( pT );

		pT = new boost::thread( _BuildSystem, this, 0, m_nImgWidth, m_nImgHeight / 4, 3 * m_nImgHeight / 8 );

		m_ThreadGrp.add_thread( pT );

		pT = new boost::thread( _BuildSystem, this, 0, m_nImgWidth, 3 * m_nImgHeight / 8, m_nImgHeight / 2 );

		m_ThreadGrp.add_thread( pT );

		pT = new boost::thread( _BuildSystem, this, 0, m_nImgWidth, m_nImgHeight / 2, 5 * m_nImgHeight / 8 );

		m_ThreadGrp.add_thread( pT );

		pT = new boost::thread( _BuildSystem, this, 0, m_nImgWidth, 5 * m_nImgHeight / 8, 3 * m_nImgHeight / 4 );

		m_ThreadGrp.add_thread( pT );

		pT = new boost::thread( _BuildSystem, this, 0, m_nImgWidth, 3 * m_nImgHeight / 4, 7 * m_nImgHeight / 8 );

		m_ThreadGrp.add_thread( pT );

		_BuildSystem( this, 0, m_nImgWidth, 7 * m_nImgHeight / 8, m_nImgHeight );

	    // wait for all threads to finish
		m_ThreadGrp.join_all();
	}
	else {
		// 1 thread or an invalid number was requested
		_BuildSystem( this, 0, m_nImgWidth, 0, m_nImgHeight );
	}

    // calculate deltaPose as Hinv * Jt * error
    // deltaPose = LHS.inverse() * RHS;
    Eigen::Vector6d Delta;

    Delta = m_LHS.ldlt().solve( m_RHS );

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
	_FlipImg( m_vVirtImg );
	m_pVirtCam->CaptureDepth( m_vVirtDepth.data() );
	_FlipDepth( m_vVirtDepth );
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
            Eigen::Vector3d Pv;    // 3d point
            Eigen::Vector4d Ph;        // homogenized point

            // check if pixel is contained in our model (i.e. has depth)
            if( pLS->m_vVirtDepth[ii * pLS->m_nImgWidth + jj] == 0 ) {
                continue;
            }

            // --------------------- first term 1x2
            // evaluate 'a' = L[ Trv * Linv( Uv ) ]
			// since we are moving the virtual camera as we update it
			// the projection falls in the same place
			// thus we can compare the intensities directly
			float fDepth = pLS->m_vVirtDepth[ii * pLS->m_nImgWidth + jj];
            Pv = pLS->_BackProject( jj, ii, fDepth );

            // project onto reference camera
            Eigen::Vector3d Lr;
			Lr << jj, ii, 1;
			Lr = Lr * fDepth;

			Ur << jj, ii;

            // check if point falls in camera's field of view
            if( (Ur( 0 ) <= 1) || (Ur( 0 ) >= pLS->m_nImgWidth - 2) || (Ur( 1 ) <= 1)
                    || (Ur( 1 ) >= pLS->m_nImgHeight - 2) ) {
                continue;
            }

            // finite differences
            Eigen::Matrix<double, 1, 2> Term1;

            Term1( 0 ) = pLS->m_vFDImgRL[ii * pLS->m_nImgWidth + jj];
            Term1( 1 ) = pLS->m_vFDImgBT[ii * pLS->m_nImgWidth + jj];

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
            Term3i    = pLS->m_Gen[0] * Ph;
            Term3     = Term3i.head( 3 );
            J( 0, 0 ) = Term1 * Term2 * Term3;
            Term3i    = pLS->m_Gen[1] * Ph;
            Term3     = Term3i.head( 3 );
            J( 0, 1 ) = Term1 * Term2 * Term3;
            Term3i    = pLS->m_Gen[2] * Ph;
            Term3     = Term3i.head( 3 );
            J( 0, 2 ) = Term1 * Term2 * Term3;
            Term3i    = pLS->m_Gen[3] * Ph;
            Term3     = Term3i.head( 3 );
            J( 0, 3 ) = Term1 * Term2 * Term3;
            Term3i    = pLS->m_Gen[4] * Ph;
            Term3     = Term3i.head( 3 );
            J( 0, 4 ) = Term1 * Term2 * Term3;
            Term3i    = pLS->m_Gen[5] * Ph;
            Term3     = Term3i.head( 3 );
            J( 0, 5 ) = Term1 * Term2 * Term3;

            // estimate LHS (Hessian)
            // LHS = Hessian = Jt * J
            LHS += J.transpose() * J;

            // std::cout << "J is: " << std::endl << J << std::endl;
            // estimate RHS (error)
            // RHS = Jt * e
            RHS += J.transpose()
                   * (pLS->m_vRefImg[ii * pLS->m_nImgWidth + jj] - pLS->m_vVirtImg[ii * pLS->m_nImgWidth + jj]);

            // calculate mean pixel error
            Error += fabs( pLS->m_vRefImg[ii * pLS->m_nImgWidth + jj] - pLS->m_vVirtImg[ii * pLS->m_nImgWidth + jj] );

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

// //////////////////////////////////////////////////////////////////////////////
inline Eigen::Vector3d LinearSystem::_Project(
        const Eigen::Vector3d& P
        )
{
    Eigen::Vector3d T = P;

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
inline void LinearSystem::_RGB2Gray(
        const std::vector<unsigned char>& RGB,    // < Input: RGB image
        Eigen::VectorXf&                  Gray    // < Output: Grayscale image
        )
{
    unsigned int Idx;

    for( int ii = 0; ii < m_nImgHeight; ii++ ) {
        for( int jj = 0; jj < m_nImgWidth; jj++ ) {
            // with flipping
            Idx                         = (m_nImgHeight - ii - 1) * m_nImgWidth * 3 + jj * 3;
            Gray[ii * m_nImgWidth + jj] = float(RGB[Idx] + RGB[Idx + 1] + RGB[Idx + 2]) / 3.0;

            // without flipping
            // Idx = ii * g_nImgWidth * 3 + jj * 3;
            // Gray[ii * g_nImgWidth + jj] = float(RGB[Idx] + RGB[Idx + 1] + RGB[Idx + 2] ) / 3.0;
        }
    }
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