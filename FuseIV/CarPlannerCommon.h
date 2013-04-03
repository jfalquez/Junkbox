#ifndef CARPLANNERCOMMON_H
#define CARPLANNERCOMMON_H
#define DEBUG 1

#include "Eigen/Eigen"
#include <Eigen/Dense>
#include <Eigen/SparseCore>
//#include <Eigen/StdVector>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <sys/time.h>
#include <time.h>
#include <boost/timer/timer.hpp>
#include <boost/format.hpp>

#ifdef __APPLE__
#include "pthread.h"
#include <mach/clock.h>
#include <mach/mach.h>
#define SetThreadName(x) pthread_setname_np(x);
#else
#include <sys/prctl.h>
#define SetThreadName(x) prctl(PR_SET_NAME,x,0,0,0);
#endif

#define DEBUG 1
#ifdef DEBUG
#define dout(str) std::cout << __FUNCTION__ << " --  " << str << std::endl
#else
#define dout(str)
#endif

#define CAR_HEIGHT_OFFSET 0.1
#define VICON_CAR_HEIGHT_OFFSET 0.11


enum MochaCommands{
    eMochaPause = 1,
    eMochaSolve = 2,
    eMochaRestart = 3,
    eMochaStep = 4,
    eMochaLearn = 5,
    eMochaToggleTrajectory = 6,
    eMochaTogglePlans = 7,
    eMochaLoad = 8,
    eMochaClear = 9,
    eMochaFixGround = 10,

    eMochaLeft = 90,
    eMochaRight = 91,
    eMochaUp = 92,
    eMochaDown = 93
};

enum OptimizationTask {
    eGaussNewton = 0,
    eDiscrete = 1,
    eDiscreteSearch = 2
};

enum eLocType { VT_AIR, VT_GROUND, VT_REC_RAMP, VT_CIR_RAMP };

// add Vector5d to eigen namespace
#define CURVE_DIM 4 //number of dimensions for the curve
#define ACCEL_INDEX 4 //the index of the acceleration
#define POSE2D_DIM 4
#define POSE_DIM 5
namespace Eigen {
    //typedef Eigen::Matrix<double, 5, 1 >  Vector5d;
    //typedef Eigen::Matrix<double, 6, 1 >  Vector6d;
    typedef Matrix<double,6,1> VectorPose3D; // defined as X,Y,Theta,V,Curvature
    typedef Matrix<double,POSE_DIM,1> VectorPose; // defined as X,Y,Theta,V,Curvature
    typedef Matrix<double,CURVE_DIM,1> VectorCubic2D;
    typedef Matrix<double,POSE2D_DIM,1> VectorPose2D;
    typedef Matrix<double, POSE_DIM-1,POSE_DIM-1> MatrixJtJ;
    typedef Matrix<double, POSE_DIM-1,POSE_DIM-1> MatrixWeights;
    typedef Matrix<double, 3,4> MatrixCubicJac;


    typedef Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> MatrixXdRowMaj;
    typedef Matrix<double,5,1> Vector5d ;
    typedef Matrix<double,6,1> Vector6d ;
    typedef Matrix<double,7,1> Vector7d ;

    typedef Matrix<double,6,6> Matrix6d;

    typedef std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d> > Vector6dAlignedVec;
    typedef std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > Vector3dAlignedVec;
    typedef std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d> > Vector4dAlignedVec;
    typedef std::vector<Eigen::Vector7d,Eigen::aligned_allocator<Eigen::Vector7d> > Vector7dAlignedVec;
    typedef std::vector<Eigen::VectorXd,Eigen::aligned_allocator<Eigen::VectorXd> > VectorXdAlignedVec;
    typedef std::vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d> > Matrix4dAlignedVec;
    typedef std::vector<Eigen::MatrixXdRowMaj,Eigen::aligned_allocator<Eigen::MatrixXdRowMaj> > MatrixXdRowMajAlignedVec;

    typedef std::vector<std::pair<Eigen::VectorPose,Eigen::VectorPose> > ErrorVec;
}


inline void current_utc_time(struct timespec *ts) {

#ifdef __MACH__ // OS X does not have clock_gettime, use clock_get_time
  clock_serv_t cclock;
  mach_timespec_t mts;
  host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
  clock_get_time(cclock, &mts);
  mach_port_deallocate(mach_task_self(), cclock);
  ts->tv_sec = mts.tv_sec;
  ts->tv_nsec = mts.tv_nsec;
#else
  clock_gettime(CLOCK_REALTIME, ts);
#endif

}




//static boost::timer::cpu_timer g_cpuTimer;

// Aux Time Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline double Tic() {

    struct timeval tv;
    gettimeofday(&tv, 0);
    return tv.tv_sec  + 1e-6 * (tv.tv_usec);

    //struct timespec tv;
    //current_utc_time(&tv);
    //return tv.tv_sec + 1e-9 * (tv.tv_nsec);

    //boost::timer::cpu_times const elapsed_times(g_cpuTimer.elapsed());
    //boost::timer::nanosecond_type const elapsed(elapsed_times.system+ elapsed_times.user);
    //return elapsed*1e9;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline double RealTime() {
    return Tic();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline double Toc(double dTic) {
    return Tic() - dTic;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline double TocMS(double dTic) {
    return ( Tic() - dTic)*1000.;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline double powi(double num, int exp) {
    if(exp == 0 ){
        return 1;
    }else if( exp < 0 ) {
        return 0;
    }else{
        double ret = num;
        for(int ii = 1; ii < exp ; ii++) {
            ret *= num;
        }
        return ret;
    }
}

////////////////////////////////////////////////////////////////////////////

template <class Derived> std::fstream& operator>>(std::fstream& fs, Eigen::MatrixBase<Derived>& mM) {
    typedef typename Eigen::internal::traits<Derived>::Scalar LScalar;
    std::string sLine;
    int nHeight = 0;
    int nWidth;
    std::vector<LScalar> vVals;
    LScalar dVal;
    while (getline(fs, sLine)) {
        std::stringstream ss(std::stringstream::in |
                std::stringstream::out);
        ss << sLine;
        int count = 0;
        while (ss >> dVal) {
            count++;
            vVals.push_back(dVal);
        }

        if (nHeight == 0) nWidth = vVals.size();
        nHeight++;
    }
    if (int(vVals.size()) != nWidth * nHeight) {
        printf("ERROR: while reading matrix from file, missing data.\n");
        return fs;
    }
    mM = Derived(nHeight, nWidth);
    for (int nRow = 0; nRow < nHeight; nRow++) {
        for (int nCol = 0; nCol < nWidth; nCol++) {
            mM(nRow, nCol) = vVals[ nRow * nWidth + nCol ];
        }
    }
    return fs;
}

////////////////////////////////////////////////////////////////////////////
inline Eigen::Matrix4d Tinv(const Eigen::Matrix4d& T)
{
    Eigen::Matrix4d res = T;
    res.block<3,3>(0,0) = T.block<3,3>(0,0).transpose();
    res.block<3,1>(0,3) = -(T.block<3,3>(0,0).transpose())*T.block<3,1>(0,3);
    return res;
}

////////////////////////////////////////////////////////////////////////////
inline Eigen::Matrix3d Tinv_2D(const Eigen::Matrix3d& T)
{
    Eigen::Matrix3d res = T;
    res.block<2,2>(0,0) = T.block<2,2>(0,0).transpose();
    res.block<2,1>(0,2) = -(T.block<2,2>(0,0).transpose())*T.block<2,1>(0,2);
    return res;
}

////////////////////////////////////////////////////////////////////////////
inline Eigen::Matrix3d Cart2T_2D(const Eigen::Vector3d& cart)
{
    Eigen::Matrix3d res;
    res << cos(cart[2]), -sin(cart[2]), cart[0],
           sin(cart[2]),  cos(cart[2]), cart[1],
           0           ,  0           ,       1;
    return res;
}

////////////////////////////////////////////////////////////////////////////
inline Eigen::Vector3d T2Cart_2D(const Eigen::Matrix3d& T)
{
    Eigen::Vector3d res;
    res[0] = T(0,2);
    res[1] = T(1,2);
    res[2] = atan2(T(1,0),T(0,0));
    return res;
}

////////////////////////////////////////////////////////////////////////////
inline Eigen::Matrix4d Cart2Tinv(const Eigen::Vector6d& v)
{
    double x = v(0),y = v(1),z = v(2), p = v(3), q = v(4), r = v(5);
    double cq, cp, cr, sq, sp, sr;
    cr = cos( r );
    cp = cos( p );
    cq = cos( q );

    sr = sin( r );
    sp = sin( p );
    sq = sin( q );

    Eigen::Matrix4d T;
    T <<         cq*cr,          cq*sr,   -sq,                          z*sq-x*cq*cr-y*cq*sr,
        cr*sp*sq-cp*sr, cp*cr+sp*sq*sr, cq*sp, x*(cp*sr-cr*sp*sq)-y*(cp*cr+sp*sq*sr)-z*cq*sp,
        sp*sr+cp*cr*sq, cp*sq*sr-cr*sp, cp*cq, y*(cr*sp-cp*sq*sr)-x*(sp*sr+cp*cr*sq)-z*cp*cq,
                     0,              0,     0,                                             1;
    return T;
}

inline Eigen::MatrixXdRowMajAlignedVec dCart2T(const Eigen::Vector6d& v)
{
    double x = v(0),y = v(1),z = v(2), p = v(3), q = v(4), r = v(5);
    double cq, cp, cr, sq, sp, sr;
    cr = cos( r );
    cp = cos( p );
    cq = cos( q );

    sr = sin( r );
    sp = sin( p );
    sq = sin( q );

    Eigen::MatrixXdRowMajAlignedVec  vJ;
    vJ.reserve(6);
    vJ.push_back(Eigen::Matrix4d::Zero());
    vJ.back()(0,3) = 1.0;   //x derivative

    vJ.push_back(Eigen::Matrix4d::Zero());
    vJ.back()(1,3) = 1.0;   //y derivative

    vJ.push_back(Eigen::Matrix4d::Zero());
    vJ.back()(2,3) = 1.0;   //z derivative

    Eigen::Matrix4d dJdP; //p derivative;
    dJdP << 0, sp*sr+cp*cr*sq,   cp*sr-cr*sp*sq,   0,
            0, cp*sq*sr-cr*sp,  -cp*cr-sp*sq*sr,   0,
            0,         cp*cp,            -cp*sp,   0,
            0,              0,                0,   0;
    vJ.push_back(dJdP);

    Eigen::Matrix4d dJdQ; //q derivative;
    dJdQ << -cr*sq,  cp*cr*sp, cp*cp*cr, 0,
            -sq*sr,  cp*sp*sr, cp*cp*sr, 0,
              -cp ,    -sp*sq,   -cp*sq, 0,
                0 ,       0  ,        0, 0;
    vJ.push_back(dJdQ);

    Eigen::Matrix4d dJdR; //r derivative;
    dJdR << -cp*sr,  -cp*cr-sp*sq*sr,  cr*sp-cp*sq*sr, 0,
            cp*cr ,  cr*sp*sq-cp*sr ,  sp*sr+cp*cr*sq, 0,
                0 ,                0,               0, 0,
                0 ,                0,               0, 0;
    vJ.push_back(dJdR);
    return vJ;
}


inline Eigen::MatrixXdRowMajAlignedVec dCart2Tinv(const Eigen::Vector6d& v)
{
    double x = v(0),y = v(1),z = v(2), p = v(3), q = v(4), r = v(5);
    double cq, cp, cr, sq, sp, sr;
    cr = cos( r );
    cp = cos( p );
    cq = cos( q );

    sr = sin( r );
    sp = sin( p );
    sq = sin( q );

    Eigen::MatrixXdRowMajAlignedVec vJ;
    vJ.reserve(6);
    Eigen::Matrix4d dJdX; //x derivative;
    dJdX << 0, 0, 0,           -cq*cr,
            0, 0, 0,   cp*sr-cr*sp*sq,
            0, 0, 0,  -sp*sr-cp*cr*sq,
            0, 0, 0,                0;
    vJ.push_back(dJdX);


    Eigen::Matrix4d dJdY; //x derivative;
    dJdY << 0, 0, 0,           -cq*sr,
            0, 0, 0,  -cp*cr-sp*sq*sr,
            0, 0, 0,   cr*sp-cp*sq*sr,
            0, 0, 0,                0;
    vJ.push_back(dJdY);

    Eigen::Matrix4d dJdZ; //x derivative;
    dJdZ << 0, 0, 0,     sq,
            0, 0, 0, -cq*sp,
            0, 0, 0, -cp*cq,
            0, 0, 0,      0;
    vJ.push_back(dJdZ);

    Eigen::Matrix4d dJdP; //p derivative;
    dJdP <<                0,                0,      0,                                             0,
            sp*sr + cp*cr*sq,   cp*sq*sr-cr*sp,  cp*cq, y*(cr*sp-cp*sq*sr)-x*(sp*sr+cp*cr*sq)-z*cp*cq,
            cp*sr - cr*sp*sq,  -cp*cr-sp*sq*sr, -cq*sp, y*(cp*cr+sp*sq*sr)-x*(cp*sr-cr*sp*sq)+z*cq*sp,
                           0,                0,      0,                                             0;
    vJ.push_back(dJdP);

    Eigen::Matrix4d dJdQ; //q derivative;
    dJdQ <<   -cr*sq,    -sq*sr,     -cq,           z*cq+x*cr*sq+y*sq*sr,
            cq*cr*sp,  cq*sp*sr,  -sp*sq,  z*sp*sq-x*cq*cr*sp-y*cq*sp*sr,
            cp*cq*cr,  cp*cq*sr,  -cp*sq,  z*cp*sq-x*cp*cq*cr-y*cp*cq*sr,
                   0,         0,       0,                              0;
    vJ.push_back(dJdQ);

    Eigen::Matrix4d dJdR; //r derivative;
    dJdR <<          -cq*sr,            cq*cr, 0,                           x*cq*sr-y*cq*cr,
            -cp*cr-sp*sq*sr,   cr*sp*sq-cp*sr, 0,     x*(cp*cr+sp*sq*sr)+y*(cp*sr-cr*sp*sq),
             cr*sp-cp*sq*sr,   sp*sr+cp*cr*sq, 0,    -x*(cr*sp-cp*sq*sr)-y*(sp*sr+cp*cr*sq),
                          0,                0, 0,                                         0;
    vJ.push_back(dJdR);
    return vJ;
}

inline Eigen::SparseMatrix<double> dT2Cart(const Eigen::Matrix4d& T)
{
    Eigen::SparseMatrix<double> J(6,16);
    J.coeffRef(0,3) = 1.0; //x derivative
    J.coeffRef(1,7) = 1.0; //y derivative
    J.coeffRef(2,11) = 1.0; //z derivative

    J.coeffRef(3,9) = T(2,2)/(powi(T(2,1),2) + powi(T(2,2),2)); //atan2 -> T22 derivative
    J.coeffRef(3,10) = -T(2,1)/(powi(T(2,1),2) + powi(T(2,2),2)); //atan2 -> r21 derivative

    J.coeffRef(4,8) = -1/sqrt(1 - pow(T(2,0),2)); //-asin -> r20 derivative

    J.coeffRef(5,4) = T(0,0)/(powi(T(1,0),2) + powi(T(0,0),2)); //atan2 -> r10 derivative
    J.coeffRef(5,0) = -T(1,0)/(powi(T(1,0),2) + powi(T(0,0),2)); //atan2 -> r10 derivative
    return J;
}


#endif // CARPLANNERCOMMON_H
