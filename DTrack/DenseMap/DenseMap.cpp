#include <queue>

#include <Mvlpp/Mvl.h>

#include "DenseMap.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
DenseMap::DenseMap()
{
    m_dLastModifiedTime = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
DenseMap::~DenseMap()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// allocate a new frame, but do not link it into the graph
FramePtr DenseMap::NewFrame(
        double          dTime,                  //< Input: Sensor time
        const cv::Mat&  GreyImage,              //< Input: Greyscale image
        const cv::Mat&  DepthImage,             //< Input: Depth image
        const cv::Mat&  GreyThumb,              //< Input: Greyscale thumbnail
        const cv::Mat&  DepthThumb              //< Input: Depth thumbnail
    )
{
    FramePtr pFrame( new ReferenceFrame );
    pFrame->SetId( m_vFrames.size() );
    pFrame->SetTime( dTime );
    pFrame->SetImages( GreyImage, DepthImage );
    pFrame->SetThumbs( GreyThumb, DepthThumb );
    m_vFrames.push_back( pFrame );
    _UpdateModifiedTime();
    return pFrame;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DenseMap::FindEdgeId(
        unsigned int        uStartId,       //< Input:
        unsigned int        uEndId,         //< Input:
        unsigned int&       uEdgeId         //< Output: global ID of edge
    )
{
    FramePtr pf = GetFramePtr( uStartId );
    std::vector<unsigned int>& vEdgeIds = pf->Neighbors();
    for( size_t ii = 0; ii < vEdgeIds.size(); ++ii ){
        EdgePtr pEdge = m_vEdges[ vEdgeIds[ii] ];
        if( pEdge->EndId() == uEndId   && pEdge->StartId() == uStartId ){
            uEdgeId =  vEdgeIds[ii];
            return true;
        }
        else if( pEdge->EndId() == uStartId   && pEdge->StartId() == uEndId ){
            uEdgeId =  vEdgeIds[ii];
            return true;
        }
    }
    printf("ERROR: edge not found!\n" );
    return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DenseMap::LinkFrames(
        FramePtr                    pA,         //< Input:
        FramePtr                    pB,         //< Input:
        const Eigen::Matrix4d&      Tab         //< Input:
    )
{

    assert( pA->Id() < m_vFrames.size() );
    assert( pB->Id() < m_vFrames.size() );
    assert( pA->Id() == m_vFrames[pA->Id()]->Id() );
    assert( pB->Id() == m_vFrames[pB->Id()]->Id() );

    EdgePtr pEdge( new TransformEdge( pA->Id(), pB->Id(), Tab ) );
    // make sure we're not duplicating an existing edge
    //        if( HasEdge( nStartId, nEndId ) ){
    //            printf("ERROR: edge <nStartId,nEndId> or <nEndId,nStartId> already exists\n");
    //        }
    unsigned int nEdgeId = m_vEdges.size();

    pA->AddNeighbor( nEdgeId );
    pB->AddNeighbor( nEdgeId );
    pB->SetParentEdgeId( nEdgeId );
    _UpdateModifiedTime();
    m_vEdges.push_back( pEdge );
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DenseMap::FrameExists(
        unsigned int    uId     //< Input:
    )
{
    return uId < m_vFrames.size() && m_vFrames[uId] != NULL;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DenseMap::EdgeExists(
        unsigned int    uStartId,       //< Input:
        unsigned int    uEndId          //< Input:
    )
{
    unsigned int tmp;
    return FindEdgeId( uStartId, uEndId, tmp );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
EdgePtr DenseMap::GetEdgePtr( unsigned int uEdgeId )
{
    assert( uEdgeId < m_vEdges.size() );
    return m_vEdges[ uEdgeId ];
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
EdgePtr DenseMap::GetEdgePtr( unsigned int uStartId, unsigned int uEndId )
{
    if( FrameExists( uStartId ) && FrameExists( uEndId ) ){
        FramePtr pf = GetFramePtr( uStartId );
        unsigned int uEdgeId;
        if( FindEdgeId( uStartId, uEndId, uEdgeId ) ){
            assert( uEdgeId < m_vEdges.size() );
            return m_vEdges[ uEdgeId ];
        }
    }
    return EdgePtr( (TransformEdge*)NULL ); // like null
}

/*
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
FramePtr DenseMap::GetFirstFramePtr()
{
    if( m_vFrames.size() > 0) {
        return m_vFrames.front();
    }else {
        return FramePtr( (ReferenceFrame*)NULL );
    }
}
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
FramePtr DenseMap::GetFramePtr( unsigned int uFrameId )
{
    std::vector<FramePtr>::reverse_iterator rit;
    for( rit =  m_vFrames.rbegin(); rit != m_vFrames.rend(); ++rit ) {
        if( (*rit)->Id() == uFrameId )
            return (*rit);
    }

    return FramePtr( (ReferenceFrame*)NULL );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DenseMap::SetRelativeTransform( int nStartId, int nEndId, Eigen::Matrix4d& Tab )
{
    EdgePtr pEdge = GetEdgePtr( nStartId, nEndId );
    pEdge->SetTransform( nStartId, nEndId, Tab );
    _UpdateModifiedTime();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DenseMap::GetRelativeTransform( int nStartId, int nEndId, Eigen::Matrix4d& Tab )
{
    EdgePtr pEdge = GetEdgePtr( nStartId, nEndId );
    return pEdge->GetTransform( nStartId, nEndId, Tab );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DenseMap::GetTransformFromParent(
        unsigned int        uChildFrameId,  //< Input: Child ID we will find parent of
        Eigen::Matrix4d&    dTab            //< Output: Found transform from parent to child
    )
{
    FramePtr pChildFrame  = GetFramePtr( uChildFrameId );
    FramePtr pParentFrame = GetFramePtr( pChildFrame->ParentEdgeId() );
    if( pParentFrame != NULL && pChildFrame != NULL ) {
        if( GetRelativeTransform( pParentFrame->Id(), uChildFrameId, dTab ) ) {
            return true;
        }
    }
    dTab = Eigen::Matrix4d::Identity();
    return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
FramePtr DenseMap::GetParentFramePtr( unsigned int uChildFrameId )
{
    FramePtr pChildFrame = GetFramePtr( uChildFrameId );
    if( pChildFrame->ParentEdgeId() == NO_PARENT ) {
        return FramePtr( (ReferenceFrame*)NULL );
    }
    EdgePtr pEdge = GetEdgePtr( pChildFrame->ParentEdgeId() );
    FramePtr pParentFrame;
    pParentFrame = pEdge->EndId() == uChildFrameId ? GetFramePtr(pEdge->StartId()) : GetFramePtr(pEdge->EndId());
    return pParentFrame;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DenseMap::SetPathBasePose(
        const Eigen::Matrix4d&      Pose        //< Input: Base pose of path
    )
{
    m_dBasePose = Pose;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DenseMap::AddPathPose(
        const Eigen::Matrix4d&      Tab         //< Input: Relative transform
    )
{
    m_vPath.push_back( Tab );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
FramePtr DenseMap::GetCurrentKeyframe()
{
    return m_pCurKeyframe;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DenseMap::CopyMapChanges(
        DenseMap&       rRHS        //< Input: Map to copy from
    )
{
    if( m_dLastModifiedTime >= rRHS.m_dLastModifiedTime ){
        return false; // nothing to do, map is up-to date
    }

    // make sure we're dealing with the same map?
//        assert( m_nMapId == rRHS.m_nMapId );

    // find data we need to update.  All we need to do here is search in the SLAM graph until
    // we stop finding nodes (Landmarks or Frames) that have been modified since m_dLastModifiedTime
//        _DFSCopy( )
    // HACK TODO FIXME  -- just to get going, copy the last 100 frames as a quick hack
    m_vEdges.resize( rRHS.m_vEdges.size() );
    m_vFrames.resize( rRHS.m_vFrames.size() );
    m_vPath.resize( rRHS.m_vPath.size() );
    m_pCurKeyframe = rRHS.m_pCurKeyframe;

    for( int ii = rRHS.m_vEdges.size()-1; ii >= std::max( (int)rRHS.m_vEdges.size()-5, 0 ); ii-- ){
        m_vEdges[ii] = boost::shared_ptr<TransformEdge>( new TransformEdge( *rRHS.m_vEdges[ii] ) );
        //m_vEdges[ii] = rRHS.m_vEdges[ii]; // will do a deep copy
    }

    for( int ii = rRHS.m_vFrames.size()-1; ii >= std::max( (int)rRHS.m_vFrames.size()-5, 0 ); ii-- ){
        m_vFrames[ii] = boost::shared_ptr<ReferenceFrame>( new ReferenceFrame( *rRHS.m_vFrames[ii] ) );
        //m_vFrames[ii] = rRHS.m_vFrames[ii]; // will do a deep copy
    }

    for( int ii = rRHS.m_vPath.size()-1; ii >= std::max( (int)rRHS.m_vPath.size()-5, 0 ); ii-- ){
        m_vPath[ii] = rRHS.m_vPath[ii];
    }


    m_dLastModifiedTime = rRHS.m_dLastModifiedTime;
    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DenseMap::SetKeyframe(
        FramePtr    pKeyframe       //< Input: Pointer to keyframe
    )
{
    m_pCurKeyframe = pKeyframe;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DenseMap::Print()
{
    printf("*******************************************\n");
    printf("EDGES\n");
    printf("*******************************************\n");
    for( unsigned int ii=0; ii < m_vEdges.size(); ii++ ) {
        printf("Id: %d  start_frame: %d  end_frame: %d \n", ii, m_vEdges[ii]->StartId(), m_vEdges[ii]->EndId());
    }

    printf("*******************************************\n");
    printf("FRAMES\n");
    printf("*******************************************\n");
    for( unsigned int ii=0; ii < m_vFrames.size(); ii++ ) {
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DenseMap::_UpdateModifiedTime()
{
    m_dLastModifiedTime = mvl::Tic();
}
