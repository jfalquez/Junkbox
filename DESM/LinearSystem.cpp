/*
 * File:   LinearSystem.cpp
 * Author: jmf
 *
 * Created on July 29, 2012, 1:44 PM
 */

#include "LinearSystem.h"
#include "se3.h"
#include "so3.h"
#include <Mvlpp/Mvl.h>

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
        GLSimCam* RefCam,
        GLSimCam* VirtCam
        )
 {
    // store parameters
    m_nImgWidth  = RefCam->ImageWidth();
    m_nImgHeight = RefCam->ImageHeight();

    // store K matrix in robotics frame
    // permutation matrix
    Eigen::Matrix3d M;

    M << 0, 1, 0, 0, 0, 1, 1, 0, 0;

    m_Kv = RefCam->GetKMatrix();
    m_Kr = m_Kv * M;

    // capture images
    std::vector<unsigned char> vRGB;

    // resize vectors
    vRGB.resize( m_nImgWidth * m_nImgHeight * 3 );
    m_vRefImg.resize( m_nImgWidth * m_nImgHeight );
    m_vVirtImg.resize( m_nImgWidth * m_nImgHeight );
    m_vVirtDepth.resize( m_nImgWidth * m_nImgHeight );

    // populate vectors & convert to greyscale
    RefCam->CaptureRGB( vRGB );
    _RGB2Gray( vRGB, m_vRefImg );
    VirtCam->CaptureRGB( vRGB );
    _RGB2Gray( vRGB, m_vVirtImg );
    VirtCam->CaptureDepth( m_vVirtDepth.data() );
    _FlipDepth( m_vVirtDepth );

    // print initial error
    Eigen::VectorXf ImgError;

    ImgError = m_vRefImg - m_vVirtImg;

    // initialize estimate
    m_dTrv = Eigen::Matrix4d::Identity();

    // print initial error
    m_nErrorPts = m_nImgHeight * m_nImgWidth;
    m_dError    = ImgError.lpNorm<1>();

    std::cout << "Error is: " << Error() << std::endl;
}

// //////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4d LinearSystem::Solve()
 {
    // reset LHS + RHS
    m_LHS.setZero();
    m_RHS.setZero();

    // reset error
    m_dError    = 0;
    m_nErrorPts = 0;

    _BuildSystem();

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
double LinearSystem::Error()
 {
    return m_dError / m_nErrorPts;
}

/**************************************************************************************************
 *
 * PRIVATE METHODS
 *
 **************************************************************************************************/
void LinearSystem::_BuildSystem()
 {
    // Jacobian
    Eigen::Matrix<double, 1, 6> J;

    J.setZero();

    for( int ii = 0; ii < m_nImgHeight; ii++ ) {
        for( int jj = 0; jj < m_nImgWidth; jj++ ) {
            // variables
            Eigen::Vector2d Ur;        // pixel position
            Eigen::Vector3d Pr, Pv;    // 3d point
            Eigen::Vector4d Ph;        // homogenized point

            // check if pixel is contained in our model (i.e. has depth)
            if( m_vVirtDepth[ii * m_nImgWidth + jj] == 0 ) {
                continue;
            }

            // --------------------- first term 1x2
            // evaluate 'a' = L[ Trv * Linv( Uv ) ]
            // back project to virtual camera's reference frame
            // this already brings points to robotics reference frame
            Pv = _BackProject( jj, ii, m_vVirtDepth[ii * m_nImgWidth + jj] );

            // convert to homogeneous coordinate
            Ph << Pv, 1;

            // transform point to reference camera's frame
            // Pr = Trv * Pv
            Ph = m_dTrv * Ph;
            Pr = Ph.head( 3 );

            // project onto reference camera
            Eigen::Vector3d Lr;

            Lr = _Project( Pr );
            Ur = Lr.head( 2 );
            Ur = Ur / Lr( 2 );

            // check if point falls in camera's field of view
            if( (Ur( 0 ) <= 1) || (Ur( 0 ) >= m_nImgWidth - 2) || (Ur( 1 ) <= 1) || (Ur( 1 ) >= m_nImgHeight - 2) ) {
                continue;
            }

            // finite differences
            float                       TopPix   = _Interpolate( Ur( 0 ), Ur( 1 ) - 1, m_vRefImg );
            float                       BotPix   = _Interpolate( Ur( 0 ), Ur( 1 ) + 1, m_vRefImg );
            float                       LeftPix  = _Interpolate( Ur( 0 ) - 1, Ur( 1 ), m_vRefImg );
            float                       RightPix = _Interpolate( Ur( 0 ) + 1, Ur( 1 ), m_vRefImg );
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
            Term2         = Term2 * m_Kr;

            // --------------------- third term 3x1
            // we need Pv in homogenous coordinates
            Ph << Pv, 1;

            Eigen::Vector4d Term3i;

            // last row of Term3 is truncated since it is always 0
            Eigen::Vector3d Term3;

            // fill Jacobian with T generators
            Term3i    = m_dTrv * m_Gen[0] * Ph;
            Term3     = Term3i.head( 3 );
            J( 0, 0 ) = Term1 * Term2 * Term3;
            Term3i    = m_dTrv * m_Gen[1] * Ph;
            Term3     = Term3i.head( 3 );
            J( 0, 1 ) = Term1 * Term2 * Term3;
            Term3i    = m_dTrv * m_Gen[2] * Ph;
            Term3     = Term3i.head( 3 );
            J( 0, 2 ) = Term1 * Term2 * Term3;
            Term3i    = m_dTrv * m_Gen[3] * Ph;
            Term3     = Term3i.head( 3 );
            J( 0, 3 ) = Term1 * Term2 * Term3;
            Term3i    = m_dTrv * m_Gen[4] * Ph;
            Term3     = Term3i.head( 3 );
            J( 0, 4 ) = Term1 * Term2 * Term3;
            Term3i    = m_dTrv * m_Gen[5] * Ph;
            Term3     = Term3i.head( 3 );
            J( 0, 5 ) = Term1 * Term2 * Term3;

            // estimate LHS (Hessian)
            // LHS = Hessian = Jt * J
            m_LHS += J.transpose() * J;

            // std::cout << "J is: " << std::endl << J << std::endl;
            // estimate RHS (error)
            // RHS = Jt * e
            m_RHS += J.transpose() * (_Interpolate( Ur( 0 ), Ur( 1 ), m_vRefImg ) - m_vVirtImg[ii * m_nImgWidth + jj]);

            // calculate normalized error
            m_dError += fabs( _Interpolate( Ur( 0 ), Ur( 1 ), m_vRefImg ) - m_vVirtImg[ii * m_nImgWidth + jj] );

            m_nErrorPts++;
        }
    }
}

// //////////////////////////////////////////////////////////////////////////////
Eigen::Vector3d LinearSystem::_Project(
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
Eigen::Vector3d LinearSystem::_BackProject(
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
float LinearSystem::_Interpolate(
        const float&           X,       // < Input: X coordinate
        const float&           Y,       // < Input: Y coordinate
        const Eigen::VectorXf& Image    // < Input: Image
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
void LinearSystem::_RGB2Gray(
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
void LinearSystem::_FlipDepth(
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