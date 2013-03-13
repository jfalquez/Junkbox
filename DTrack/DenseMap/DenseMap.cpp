#include <queue>

#include <Mvlpp/Mvl.h>

#include "DenseMap.h"

////////////////////////////////////////////////////////////////////////////////////////
/// Allocate a new frame, but do not link it into the graph.
FramePtr DenseMap::NewFrame( double dTime )
{
    FramePtr pFrame( new ReferenceFrame );
    pFrame->SetId( m_vFrames.size() );
    pFrame->SetTime( dTime );
    m_vFrames.push_back( pFrame );
    return pFrame;
}

////////////////////////////////////////////////////////////////////////////////////////
// lookup the id
bool DenseMap::FindEdgeId( unsigned int uStartId,
        unsigned int uEndId,
        unsigned int& uEdgeId )  //< Output: global Id of edge
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

////////////////////////////////////////////////////////////////////////////////////////
void DenseMap::LinkFrames( FramePtr pA, FramePtr pB, Eigen::Matrix4d& Tab )
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

    m_vEdges.push_back( pEdge );
}


////////////////////////////////////////////////////////////////////////////////////////
bool DenseMap::HasFrame( unsigned int uId )
{
    return uId < m_vFrames.size() && m_vFrames[uId] != NULL;
}

////////////////////////////////////////////////////////////////////////////////////////
bool DenseMap::HasEdge( unsigned int uStartId, unsigned int uEndId )
{
    unsigned int tmp;
    return FindEdgeId( uStartId, uEndId, tmp );
}

////////////////////////////////////////////////////////////////////////////////////////
EdgePtr DenseMap::GetEdgePtr( unsigned int uStartId, unsigned int uEndId )
{
    if( HasFrame( uStartId ) && HasFrame( uEndId ) ){
        FramePtr pf = GetFramePtr( uStartId );
        unsigned int uEdgeId;
        if( FindEdgeId( uStartId, uEndId, uEdgeId ) ){
            assert( uEdgeId < m_vEdges.size() );
            return m_vEdges[ uEdgeId ];
        }
    }
    return EdgePtr(); // like null
}

////////////////////////////////////////////////////////////////////////////////////////
EdgePtr DenseMap::GetEdgePtr( unsigned int uEdgeId )
{
    assert( uEdgeId < m_vEdges.size() );
    return m_vEdges[ uEdgeId ];
}

////////////////////////////////////////////////////////////////////////////////////////
FramePtr DenseMap::GetFirstFramePtr()
{
    if( m_vFrames.size() > 0) {
        return m_vFrames.front();
    }else {
        return FramePtr( (ReferenceFrame*)NULL );
    }
}

////////////////////////////////////////////////////////////////////////////////////////
FramePtr DenseMap::GetFramePtr( unsigned int uFrameId )
{
    /*
       if( uFrameId == NO_PARENT ){
       return SlamFramePtr( (ReferenceFrame*)NULL );
       }

       assert( uFrameId < m_vFrames.size() );
       assert( m_vFrames[ uFrameId ] != NULL );
       return m_vFrames[ uFrameId ];
     */

    std::vector<FramePtr>::reverse_iterator rit;
    for( rit =  m_vFrames.rbegin(); rit != m_vFrames.rend(); ++rit ) {
        if( (*rit)->Id() == uFrameId )
            return (*rit);
    }

    return FramePtr( (ReferenceFrame*)NULL );
}

////////////////////////////////////////////////////////////////////////////////////////
//< Return: true if edge exists
bool DenseMap::GetTransformFromParent( unsigned int uChildFrameId, //< Input: Child ID we will find parent of
        Eigen::Matrix4d& dTab  )        //< Output: Found transform from parent to child
{
    FramePtr pChildFrame  = GetFramePtr( uChildFrameId );
    FramePtr pParentFrame = GetFramePtr( pChildFrame->ParentEdgeId() );
    if( pParentFrame != NULL && pChildFrame != NULL ) {
        if( GetRelativePose( pParentFrame->Id(), uChildFrameId, dTab ) ) {
            return true;
        }
    }
    dTab = Eigen::Matrix4d::Identity();
    return false;
}


////////////////////////////////////////////////////////////////////////////////////////
// set the transform between two frames:
void DenseMap::SetRelativePose( int nStartId, int nEndId, Eigen::Matrix4d& Tab )
{
    EdgePtr pEdge = GetEdgePtr( nStartId, nEndId );
    FramePtr pFrame = GetFramePtr( nStartId );
    pEdge->SetTransform( nStartId, nEndId, Tab );
}

////////////////////////////////////////////////////////////////////////////////////////
// get the transform between two frames:
bool DenseMap::GetRelativePose( int nStartId, int nEndId, Eigen::Matrix4d& Tab )
{
    EdgePtr pEdge = GetEdgePtr( nStartId, nEndId );
    FramePtr pFrame = GetFramePtr( nStartId );
    return pEdge->GetTransform( nStartId, nEndId, Tab );
}

////////////////////////////////////////////////////////////////////////////////////////
/// Return smart pointer to frames "parent"
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

////////////////////////////////////////////////////////////////////////////////////////
void DenseMap::GetPosesToDepth( std::map<unsigned int, Eigen::Matrix4d>& Poses, unsigned int nDepth )
{
    Poses.clear();

    // do BFS
    // get the first node (which will be the origin of the coordinate frame)
    FramePtr  pRoot  = m_vFrames.back();

    // now search for close nodes
    std::queue<FramePtr> q;
    pRoot->SetDepth(0);
    q.push(pRoot);

    // reset colors
    ResetNodes();

    FramePtr pCurNode;
    Eigen::Matrix4d T;

    while( !q.empty() ) {

        pCurNode = q.front();
        q.pop();
        pCurNode->SetGrey();

        // Explore neighbours
        for(unsigned int ii=0; ii < pCurNode->NumNeighbors(); ++ii) {
            unsigned int uNeighborEdgeId = pCurNode->GetNeighborEdgeId(ii);
            EdgePtr pNeighborEdge   = GetEdgePtr(uNeighborEdgeId);
            unsigned int uNeighborNodeId;

            uNeighborNodeId = ( pNeighborEdge->StartId() == pCurNode->Id() ) ?
                pNeighborEdge->EndId() : pNeighborEdge->StartId();

            FramePtr pNeighborNode = GetFramePtr(uNeighborNodeId);

            if( pCurNode->Depth() < nDepth &&
                    pNeighborNode != NULL && pNeighborNode->IsWhite()) {

                // add node to the queue
                pNeighborNode->SetDepth( pCurNode->Depth()+1 );
                pNeighborNode->SetGrey();
                q.push( pNeighborNode );
            }
        }

        // We are done with this node, add it to the map
        pCurNode->SetBlack();
        unsigned int uParentEdgeId = pCurNode->ParentEdgeId();

        if( uParentEdgeId == NO_PARENT ) {
            Poses[pCurNode->Id()] = Eigen::Matrix4d::Identity();
        }
        else {
            EdgePtr pEdge = GetEdgePtr( uParentEdgeId );
            pEdge->GetTransform(pEdge->StartId(), pEdge->EndId(), T);
            Poses[pCurNode->Id()] = T;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////
void DenseMap::GetAbsolutePosesToDepth(
         std::map<unsigned int, Eigen::Matrix4d>& absolutePoses,
         unsigned int                             nRootId,
         int                                      nDepth
        )
{
    absolutePoses.clear();

    assert( nRootId < 0 || nRootId >= m_vFrames.size() );

    // return all poses
    if(nDepth == -1 ){
        nDepth = m_vFrames.size();
    }

    FramePtr pRoot  = m_vFrames[nRootId];

    // now search for close nodes
    std::queue< FramePtr >          q;
    std::queue< Eigen::Matrix4d >   t;
    pRoot->SetDepth(0);
    q.push(pRoot);
    t.push(Eigen::Matrix4d::Identity());

    // reset colors
    ResetNodes();

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
        for(unsigned int ii=0; ii < pCurNode->NumNeighbors(); ++ii) {
            unsigned int uNeighborEdgeId = pCurNode->GetNeighborEdgeId(ii);
            EdgePtr pNeighborEdge   = GetEdgePtr(uNeighborEdgeId);
            unsigned int uNeighborNodeId;

            uNeighborNodeId = ( pNeighborEdge->StartId() == pCurNode->Id() ) ?
                                pNeighborEdge->EndId() : pNeighborEdge->StartId();

            FramePtr pNeighborNode = GetFramePtr(uNeighborNodeId);

            if( (int)pCurNode->Depth() < nDepth &&
                pNeighborNode != NULL      &&
                pNeighborNode->IsWhite()) {

                // add node to the queue
                pNeighborNode->SetDepth( pCurNode->Depth()+1 );
                pNeighborNode->SetGrey();
                q.push( pNeighborNode );
                // compute cummulative transformation (transformation from root)
                pNeighborEdge->GetTransform( pCurNode->Id(), uNeighborNodeId, T);
                t.push( curT*T );
                mvl::MakeOrthonormal( t.back() );
            }
        }

        // We are done with this node, add it to the map
        pCurNode->SetBlack();
        absolutePoses[pCurNode->Id()] = curT;

    }
}


////////////////////////////////////////////////////////////////////////////////////////
void DenseMap::ResetNodes()
{
    // reset all nodes' color to black
    std::vector<FramePtr>::iterator it;

    for(it = m_vFrames.begin(); it != m_vFrames.end(); ++it) {
        (*it)->SetWhite();
        (*it)->SetDepth(0);
    }

}


////////////////////////////////////////////////////////////////////////////////////////
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
