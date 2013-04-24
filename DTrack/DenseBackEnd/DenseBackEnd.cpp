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
    m_pMap->UpdateInternalPathFull();
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



