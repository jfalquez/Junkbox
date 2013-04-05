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
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DenseBackEnd::Init(
        DenseMap*               pMap        //< Input: Pointer to the map that should be used
    )
{
    // assign map
    m_pMap = pMap;

    return true;
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
    std::map< unsigned int, Eigen::Matrix4d >& vPath = m_pMap->GetInternalPath();
    std::vector< Sophus::SE3d  > vAbsPoses;
    vAbsPoses.reserve( vPath.size() );

    LocalParamSe3* pLocalParam = new LocalParamSe3;
    ceres::Problem Problem;
    for( unsigned int ii = 0; ii < vPath.size(); ++ii ) {
        vAbsPoses.push_back( Sophus::SE3d( vPath[ii] ) );
        Problem.AddParameterBlock( vAbsPoses.back().data(), 7, pLocalParam );
    }

    // add all edges
    for( unsigned int ii = 0; ii < m_pMap->GetNumEdges(); ++ii ) {
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
    for( unsigned int ii = 0; ii < m_pMap->GetNumEdges(); ++ii ) {
        EdgePtr pEdge = m_pMap->GetEdgePtr( ii );
        const unsigned int nStartId = pEdge->GetStartId();
        const unsigned int nEndId = pEdge->GetEndId();
        Sophus::SE3d Tse = vAbsPoses[nStartId].inverse() * vAbsPoses[nEndId];
        pEdge->SetTransform( Tse.matrix() );
    }

    m_pMap->UpdateInternalPath();
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



