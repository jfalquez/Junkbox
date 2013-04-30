#include <ceres/ceres.h>

#include "DenseBackEnd.h"
#include "AutoDiffArrayCostFunction.h"
#include "EigenCeresJetNumTraits.h"
#include "LocalParamSe3.h"


// Global CVars
DenseBackEndConfig         beConfig;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
DenseBackEnd::DenseBackEnd()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
DenseBackEnd::~DenseBackEnd()
{
    // set flag to kill all threads
    m_bRun = false;

    // wait for threads to die
    m_pPGThread->join();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DenseBackEnd::Init(
        DenseMap*               pMap        //< Input: Pointer to the map that should be used
    )
{
    // assign map
    m_pMap = pMap;

    // reset flag
    m_bRun = true;

    // start pose graph relaxation thread
    m_bDoPGRelaxation = false;
    m_pPGThread = new std::thread( _PoseGraphThread, this );

    return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DenseBackEnd::DoPoseGraphRelaxation()
{
    m_bDoPGRelaxation = true;
    m_PGCond.notify_one();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DenseBackEnd::_PoseGraphThread(
        DenseBackEnd*       pBE
    )
{
    while( pBE->m_bRun ) {

        std::unique_lock< std::mutex > lock(pBE->m_Mutex);
        while( pBE->m_bDoPGRelaxation == false ) {
            pBE->m_PGCond.wait(lock);
        }
        pBE->_PoseRelax();
        sleep(2);
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class RelEdgeCostFunc : public ceres::AutoDiffArrayCostFunction< ceres::CostFunction, RelEdgeCostFunc, 6, 7, 7 >
{
public:
    RelEdgeCostFunc( Sophus::SE3d RelTse ) { m_RelTes = RelTse.inverse(); }

    template <typename T>
    bool Evaluate(const T * const *parameters, T *residuals ) const
    {
        Eigen::Map< Eigen::Matrix<T, 6, 1> >    r(residuals);
        Eigen::Map< const Sophus::SE3Group<T> > Tws(parameters[0]);
        Eigen::Map< const Sophus::SE3Group<T> > Twe(parameters[1]);

        const Sophus::SE3Group<T> Tse = Tws.inverse() * Twe;
        Sophus::SE3Group<T> Terror = m_RelTes.cast<T>() * Tse;
        r = Terror.log();

        return true;
   }

private:
    Sophus::SE3d        m_RelTes;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DenseBackEnd::_PoseRelax()
{
    //----- START CONTENTION ZONE
    m_pMap->Lock();
    m_bDoPGRelaxation = false;
    std::map< unsigned int, Eigen::Matrix4d >& vPath = m_pMap->GetInternalPath();
    std::vector< Sophus::SE3d  > vAbsPoses;
    vAbsPoses.reserve( vPath.size() );

    LocalParamSe3* pLocalParam = new LocalParamSe3;
    ceres::Problem Problem;
    for( unsigned int ii = 0; ii < vPath.size(); ++ii ) {
        vAbsPoses.push_back( Sophus::SE3d( vPath[ii] ) );
        Problem.AddParameterBlock( vAbsPoses.back().data(), 7, pLocalParam );
    }
    const unsigned int nNumEdges = m_pMap->GetNumEdges();
    m_pMap->Unlock();
    //----- END CONTENTION ZONE

    // add all edges
    for( unsigned int ii = 0; ii < nNumEdges; ++ii ) {
        EdgePtr pEdge = m_pMap->GetEdgePtr( ii );
        const unsigned int nStartId = pEdge->GetStartId();
        const unsigned int nEndId = pEdge->GetEndId();
        Eigen::Matrix4d Tse;
        pEdge->GetOriginalTransform( nStartId, nEndId, Tse );
        RelEdgeCostFunc* pCostFunc = new RelEdgeCostFunc( Sophus::SE3d( Tse ) );
        Problem.AddResidualBlock( pCostFunc, NULL, vAbsPoses[nStartId].data(), vAbsPoses[nEndId].data() );
    }

    ceres::Solver::Options SolverOptions;
    SolverOptions.num_threads = 4;
    ceres::Solver::Summary SolverSummary;
    ceres::Solve( SolverOptions, &Problem, &SolverSummary );
    std::cout << SolverSummary.BriefReport() << std::endl;

    // copy vAbsPoses as relative transforms back to the map
    for( unsigned int ii = 0; ii < nNumEdges; ++ii ) {
        EdgePtr pEdge = m_pMap->GetEdgePtr( ii );
        const unsigned int nStartId = pEdge->GetStartId();
        const unsigned int nEndId = pEdge->GetEndId();
        Sophus::SE3d Tse = vAbsPoses[nStartId].inverse() * vAbsPoses[nEndId];
        pEdge->SetTransform( Tse.matrix() );
    }
//    m_pMap->UpdateInternalPathFull();
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class ViconAlignCostFunc : public ceres::AutoDiffArrayCostFunction< ceres::CostFunction, ViconAlignCostFunc, Sophus::SE3d::DoF, Sophus::SE3d::num_parameters >
{
public:
    ViconAlignCostFunc( Sophus::SE3d Twf1, Sophus::SE3d Twf, Sophus::SE3d Tgc ) { m_Twf1 = Twf1, m_Tfw = Twf.inverse(), m_Tgc = Tgc; }

    template <typename T>
    bool Evaluate(const T * const *parameters, T *residuals ) const
    {
        Eigen::Map< Eigen::Matrix<T, Sophus::SE3d::DoF, 1> >    r(residuals);
        Eigen::Map< const Sophus::SE3Group<T> > Tfc(parameters[0]);

        const Sophus::SE3Group<T> myTwf = m_Twf1.cast<T>() * Tfc * m_Tgc.cast<T>();
        Sophus::SE3Group<T> Terror = m_Tfw.cast<T>() * myTwf;
        r = Terror.log();

        return true;
   }

private:
    Sophus::SE3d        m_Twf1;     // first fiducial pose in Vicon world frame (this is our origin)
    Sophus::SE3d        m_Tfw;      // i'th pose in Vicon world frame (inverted)
    Sophus::SE3d        m_Tgc;      // my i'th camera poses brought from relative to a "global" frame
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DenseBackEnd::_ViconAlign()
{
    // my estimates brought to the first frame
    std::map< unsigned int, Eigen::Matrix4d > vPoses;
    m_pMap->GenerateAbsolutePoses( vPoses, 0 );

    // get first frame's Vicon pose
    FramePtr pFrame = m_pMap->GetFramePtr(0);
    Eigen::Matrix4d& Twf1 = pFrame->m_dViconPose;

    std::vector< Sophus::SE3d  > vViconPoses;
    vViconPoses.reserve( vPoses.size() );

    std::vector< Sophus::SE3d  > vMyPoses;
    vMyPoses.reserve( vPoses.size() );

    ceres::Problem Problem;

    // stuff we are solving
    Sophus::SE3d Tfc( m_pMap->m_dTfc );

    // TODO set this to member being initalized once
    LocalParamSe3* pLocalParam = new LocalParamSe3;
    Problem.AddParameterBlock( Tfc.data(), 7, pLocalParam );

    for( unsigned int ii = 0; ii < vPoses.size(); ++ii ) {

        pFrame = m_pMap->GetFramePtr(ii);
        vViconPoses.push_back( Sophus::SE3d( pFrame->m_dViconPose ) );

        vMyPoses.push_back( Sophus::SE3d( vPoses[ii] ) );

        ViconAlignCostFunc* pCostFunc = new ViconAlignCostFunc( Sophus::SE3d(Twf1), vViconPoses.back(), vMyPoses.back() );
        Problem.AddResidualBlock( pCostFunc, NULL, Tfc.data() );

    }

    ceres::Solver::Options SolverOptions;
    SolverOptions.num_threads = 4;
    ceres::Solver::Summary SolverSummary;
    ceres::Solve( SolverOptions, &Problem, &SolverSummary );
    std::cout << SolverSummary.BriefReport() << std::endl;

    // copy back results
    m_pMap->m_dTfc = Tfc.matrix();
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



