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
        unsigned int        nStartId,       //< Input:
        unsigned int        nEndId,         //< Input:
        unsigned int&       nEdgeId         //< Output: global ID of edge
    )
{
    FramePtr pf = GetFramePtr( nStartId );
    std::vector<unsigned int>& vEdgeIds = pf->GetNeighbors();
    for( size_t ii = 0; ii < vEdgeIds.size(); ++ii ){
        EdgePtr pEdge = m_vEdges[ vEdgeIds[ii] ];
        if( pEdge->GetEndId() == nEndId   && pEdge->GetStartId() == nStartId ){
            nEdgeId =  vEdgeIds[ii];
            return true;
        }
        else if( pEdge->GetEndId() == nStartId   && pEdge->GetStartId() == nEndId ){
            nEdgeId =  vEdgeIds[ii];
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

    assert( pA->GetId() < m_vFrames.size() );
    assert( pB->GetId() < m_vFrames.size() );
    assert( pA->GetId() == m_vFrames[pA->GetId()]->GetId() );
    assert( pB->GetId() == m_vFrames[pB->GetId()]->GetId() );

    EdgePtr pEdge( new TransformEdge( pA->GetId(), pB->GetId(), Tab ) );
    // make sure we're not duplicating an existing edge
    //        if( HasEdge( nGetStartId, nEndId ) ){
    //            printf("ERROR: edge <nGetStartId,nEndId> or <nEndId,nGetStartId> already exists\n");
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
        unsigned int    nId     //< Input:
    )
{
    return nId < m_vFrames.size() && m_vFrames[nId] != NULL;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DenseMap::EdgeExists(
        unsigned int    nGetStartId,    //< Input:
        unsigned int    nEndId          //< Input:
    )
{
    unsigned int tmp;
    return FindEdgeId( nGetStartId, nEndId, tmp );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
EdgePtr DenseMap::GetEdgePtr( unsigned int nEdgeId )
{
    assert( nEdgeId < m_vEdges.size() );
    return m_vEdges[ nEdgeId ];
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
EdgePtr DenseMap::GetEdgePtr( unsigned int nGetStartId, unsigned int nEndId )
{
    if( FrameExists( nGetStartId ) && FrameExists( nEndId ) ){
        unsigned int nEdgeId;
        if( FindEdgeId( nGetStartId, nEndId, nEdgeId ) ){
            assert( nEdgeId < m_vEdges.size() );
            return m_vEdges[ nEdgeId ];
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
FramePtr DenseMap::GetFramePtr( unsigned int nFrameId )
{
    std::vector<FramePtr>::reverse_iterator rit;
    for( rit =  m_vFrames.rbegin(); rit != m_vFrames.rend(); ++rit ) {
        if( (*rit)->GetId() == nFrameId )
            return (*rit);
    }

    return FramePtr( (ReferenceFrame*)NULL );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DenseMap::SetRelativeTransform( int nGetStartId, int nEndId, Eigen::Matrix4d& Tab )
{
    EdgePtr pEdge = GetEdgePtr( nGetStartId, nEndId );
    pEdge->SetTransform( nGetStartId, nEndId, Tab );
    _UpdateModifiedTime();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DenseMap::GetRelativeTransform( int nGetStartId, int nEndId, Eigen::Matrix4d& Tab )
{
    EdgePtr pEdge = GetEdgePtr( nGetStartId, nEndId );
    return pEdge->GetTransform( nGetStartId, nEndId, Tab );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DenseMap::GetTransformFromParent(
        unsigned int        nChildFrameId,  //< Input: Child ID we will find parent of
        Eigen::Matrix4d&    dTab            //< Output: Found transform from parent to child
    )
{
    FramePtr pChildFrame  = GetFramePtr( nChildFrameId );
    FramePtr pParentFrame = GetFramePtr( pChildFrame->GetParentEdgeId() );
    if( pParentFrame != NULL && pChildFrame != NULL ) {
        if( GetRelativeTransform( pParentFrame->GetId(), nChildFrameId, dTab ) ) {
            return true;
        }
    }
    dTab = Eigen::Matrix4d::Identity();
    return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
FramePtr DenseMap::GetParentFramePtr(
        unsigned int        nChildFrameId       //< Input: Child ID we will find parent of
    )
{
    FramePtr pChildFrame = GetFramePtr( nChildFrameId );
    if( pChildFrame->GetParentEdgeId() == NO_PARENT ) {
        return FramePtr( (ReferenceFrame*)NULL );
    }
    EdgePtr pEdge = GetEdgePtr( pChildFrame->GetParentEdgeId() );
    FramePtr pParentFrame;
    pParentFrame = pEdge->GetEndId() == nChildFrameId ? GetFramePtr(pEdge->GetStartId()) : GetFramePtr(pEdge->GetEndId());
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


    m_pCurKeyframe = rRHS.m_pCurKeyframe;
//    m_pCurKeyframe = boost::shared_ptr<ReferenceFrame>( new ReferenceFrame( *(rRHS.m_pCurKeyframe) ) );

    m_dBasePose = rRHS.m_dBasePose;

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
        printf("Id: %d  start_frame: %d  end_frame: %d \n", ii, m_vEdges[ii]->GetStartId(), m_vEdges[ii]->GetEndId());
    }

    printf("*******************************************\n");
    printf("FRAMES\n");
    printf("*******************************************\n");
    for( unsigned int ii=0; ii < m_vFrames.size(); ii++ ) {
    }
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DenseMap::GenerateRelativePoses(
        std::map<unsigned int, Eigen::Matrix4d>&    vPoses,     //< Output: Poses id and relative poses from input root
        int                                         nRootId,    //< Input: Frame that will be the origin [default: last frame]
        int                                         nDepth      //< Input: Grapth depth search from root [default: max depth]
    )
{
    assert( nRootId >= -1 && nRootId < m_vFrames.size() );

    // clear output vector
    vPoses.clear();

    // check if we have frames
    if( m_vFrames.empty() ) {
        return;
    }

    // return all poses
    if( nDepth == -1 ) {
        nDepth = m_vFrames.size();
    }

    // get pointer to root frame
    FramePtr pRoot = nRootId == -1? m_vFrames.back() : m_vFrames[nRootId];

    // now search for close nodes
    std::queue<FramePtr> q;
    pRoot->SetDepth(0);
    q.push(pRoot);

    // reset colors
    _ResetNodes();

    FramePtr pCurNode;
    Eigen::Matrix4d T;

    while( !q.empty() ) {

        pCurNode = q.front();
        q.pop();
        pCurNode->SetGrey();

        // Explore neighbours
        for( unsigned int ii = 0; ii < pCurNode->GetNumNeighbors(); ++ii ) {
            unsigned int uNeighborEdgeId = pCurNode->GetNeighborEdgeId(ii);
            EdgePtr pNeighborEdge   = GetEdgePtr(uNeighborEdgeId);
            unsigned int uNeighborNodeId;

            uNeighborNodeId = ( pNeighborEdge->GetStartId() == pCurNode->GetId() ) ?
                pNeighborEdge->GetEndId() : pNeighborEdge->GetStartId();

            FramePtr pNeighborNode = GetFramePtr(uNeighborNodeId);

            if( pCurNode->GetDepth() < nDepth &&
                    pNeighborNode != NULL && pNeighborNode->IsWhite()) {

                // add node to the queue
                pNeighborNode->SetDepth( pCurNode->GetDepth()+1 );
                pNeighborNode->SetGrey();
                q.push( pNeighborNode );
            }
        }

        // We are done with this node, add it to the map
        pCurNode->SetBlack();
        unsigned int uParentEdgeId = pCurNode->GetParentEdgeId();

        if( uParentEdgeId == NO_PARENT ) {
            vPoses[pCurNode->GetId()] = Eigen::Matrix4d::Identity();
        } else {
            EdgePtr pEdge = GetEdgePtr( uParentEdgeId );
            pEdge->GetTransform(pEdge->GetStartId(), pEdge->GetEndId(), T);
            vPoses[pCurNode->GetId()] = T;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DenseMap::GenerateAbsolutePoses(
        std::map<unsigned int, Eigen::Matrix4d>&    vPoses,     //< Output: Poses id and absolute poses from input root
        int                                         nRootId,    //< Input: Frame that will be the origin [default: last frame]
        int                                         nDepth      //< Input: Grapth depth search from root [default: max depth]
    )
{
    assert( nRootId >= -1 && nRootId < m_vFrames.size() );

    // clear output vector
    vPoses.clear();

    // check if we have frames
    if( m_vFrames.empty() ) {
        return;
    }

    // return all poses
    if( nDepth == -1 ) {
        nDepth = m_vFrames.size();
    }

    // get pointer to root frame
    FramePtr pRoot = nRootId == -1? m_vFrames.back() : m_vFrames[nRootId];

    // now search for close nodes
    std::queue<FramePtr>    q;
    std::queue<Eigen::Matrix4d> t;
    pRoot->SetDepth(0);
    q.push(pRoot);
    t.push(Eigen::Matrix4d::Identity());

    // reset colors
    _ResetNodes();

    FramePtr pCurNode;
    Eigen::Matrix4d curT;
    Eigen::Matrix4d T;

    while( !q.empty() ) {

        pCurNode = q.front();
        curT     = t.front();
        q.pop();
        t.pop();
        pCurNode->SetGrey();

        // Explore neighbours
        for( unsigned int ii = 0; ii < pCurNode->GetNumNeighbors(); ++ii ) {
            unsigned int uNeighborEdgeId = pCurNode->GetNeighborEdgeId(ii);
            EdgePtr pNeighborEdge   = GetEdgePtr(uNeighborEdgeId);
            unsigned int uNeighborNodeId;

            uNeighborNodeId = ( pNeighborEdge->GetStartId() == pCurNode->GetId() ) ?
                                pNeighborEdge->GetEndId() : pNeighborEdge->GetStartId();

            FramePtr pNeighborNode = GetFramePtr(uNeighborNodeId);

            if( (int)pCurNode->GetDepth() < nDepth &&
                pNeighborNode != NULL      &&
                pNeighborNode->IsWhite()) {

                // add node to the queue
                pNeighborNode->SetDepth( pCurNode->GetDepth()+1 );
                pNeighborNode->SetGrey();
                q.push( pNeighborNode );

                // compute cummulative transformation (transformation from root)
                pNeighborEdge->GetTransform( pCurNode->GetId(), uNeighborNodeId, T);
                t.push( curT*T );
                mvl::MakeOrthonormal( t.back() );
            }
        }

        // We are done with this node, add it to the map
        pCurNode->SetBlack();
        vPoses[pCurNode->GetId()] = curT;
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


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DenseMap::_ResetNodes()
{
    std::vector< FramePtr >::iterator it;

    for(it = m_vFrames.begin(); it != m_vFrames.end(); ++it) {
        (*it)->SetWhite();
        (*it)->SetDepth(0);
    }
}
