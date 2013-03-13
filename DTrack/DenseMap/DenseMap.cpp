#include <queue>

#include "Map.h"

////////////////////////////////////////////////////////////////////////////////////////
/// Allocate a new frame, but do not link it into the graph.
SlamFramePtr Map::NewFrame( double dTime )
{
    SlamFramePtr pFrame(new ReferenceFrame);
    pFrame->SetId( m_vFrames.size() );
    pFrame->SetTime( dTime );
    m_vFrames.push_back( pFrame );
    return pFrame;
}

////////////////////////////////////////////////////////////////////////////////////////
// lookup the id
bool Map::FindEdgeId( unsigned int uStartId,
        unsigned int uEndId,
        unsigned int& uEdgeId )  //< Output: global Id of edge
{
    SlamFramePtr pf = GetFramePtr( uStartId );
    std::vector<unsigned int>& vEdgeIds = pf->Neighbors();
    for( size_t ii = 0; ii < vEdgeIds.size(); ++ii ){
        SlamEdgePtr pEdge = m_vEdges[ vEdgeIds[ii] ];
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
void Map::LinkFrames( SlamFramePtr pA, SlamFramePtr pB, Eigen::Matrix4d& Tab )
{

    assert( pA->Id() < m_vFrames.size() );
    assert( pB->Id() < m_vFrames.size() );
    assert( pA->Id() == m_vFrames[pA->Id()]->Id() );
    assert( pB->Id() == m_vFrames[pB->Id()]->Id() );

    SlamEdgePtr pEdge( new TransformEdge( pA->Id(), pB->Id(), Tab ) );
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
bool Map::HasFrame( unsigned int uId )
{
    return uId < m_vFrames.size() && m_vFrames[uId] != NULL;
}

////////////////////////////////////////////////////////////////////////////////////////
bool Map::HasEdge( unsigned int uStartId, unsigned int uEndId )
{
    unsigned int tmp;
    return FindEdgeId( uStartId, uEndId, tmp );
}

////////////////////////////////////////////////////////////////////////////////////////
SlamEdgePtr Map::GetEdgePtr( unsigned int uStartId, unsigned int uEndId )
{
    if( HasFrame( uStartId ) && HasFrame( uEndId ) ){
        SlamFramePtr pf = GetFramePtr( uStartId );
        unsigned int uEdgeId;
        if( FindEdgeId( uStartId, uEndId, uEdgeId ) ){
            assert( uEdgeId < m_vEdges.size() );
            return m_vEdges[ uEdgeId ];
        }
    }
    return SlamEdgePtr(); // like null
}

////////////////////////////////////////////////////////////////////////////////////////
SlamEdgePtr Map::GetEdgePtr( unsigned int uEdgeId )
{
    assert( uEdgeId < m_vEdges.size() );
    return m_vEdges[ uEdgeId ];   
}

////////////////////////////////////////////////////////////////////////////////////////
SlamFramePtr Map::GetFirstFramePtr() 
{
    if( m_vFrames.size() > 0) {
        return m_vFrames.front();
    }else {
        return SlamFramePtr( (ReferenceFrame*)NULL );
    }
}

////////////////////////////////////////////////////////////////////////////////////////
// return reference to frame
SlamFramePtr Map::GetFramePtr( MeasurementId nId ) 
{
    return GetFramePtr( nId.m_nFrameId );
}

////////////////////////////////////////////////////////////////////////////////////////
SlamFramePtr Map::GetFramePtr( unsigned int uFrameId ) 
{
    /*
       if( uFrameId == NO_PARENT ){
       return SlamFramePtr( (ReferenceFrame*)NULL );
       }

       assert( uFrameId < m_vFrames.size() );
       assert( m_vFrames[ uFrameId ] != NULL );
       return m_vFrames[ uFrameId ];
     */

    std::vector<SlamFramePtr>::reverse_iterator rit;
    for( rit =  m_vFrames.rbegin(); rit != m_vFrames.rend(); ++rit ) {
        if( (*rit)->Id() == uFrameId )
            return (*rit);
    }

    return SlamFramePtr( (ReferenceFrame*)NULL );
}

////////////////////////////////////////////////////////////////////////////////////////
//< Return: true if edge exists
bool Map::GetTransformFromParent( unsigned int uChildFrameId, //< Input: Child ID we will find parent of
        Eigen::Matrix4d& dTab  )        //< Output: Found transform from parent to child
{
    SlamFramePtr pChildFrame  = GetFramePtr( uChildFrameId );
    SlamFramePtr pParentFrame = GetFramePtr( pChildFrame->ParentEdgeId() );
    if( pParentFrame != NULL && pChildFrame != NULL ) {
        if( GetRelativePose( pParentFrame->Id(), uChildFrameId, dTab ) ) {
            return true;
        }
    }
    dTab = Eigen::Matrix4d::Identity();
    return false;
}

////////////////////////////////////////////////////////////////////////////////////////
void Map::GetPtrsToLandmarks( std::vector<Landmark*>& rvpLandmarks )
{
    for( unsigned int ii=0; ii < m_vFrames.size(); ++ii )
    {
        std::vector<Landmark>& vLmks = m_vFrames[ii]->GetLandmarksVectorRef();
        
        for( unsigned int jj=0; jj < vLmks.size(); ++jj ){
            rvpLandmarks.push_back( &vLmks[jj] );
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////
// return reference to landmark
Eigen::Vector4d& Map::LandmarkPos( MeasurementId nId )
{
    SlamFramePtr pFrame = GetFramePtr( nId.m_nLandmarkId.m_nRefFrameId );
    return pFrame->LandmarkPos( nId.m_nLandmarkId.m_nLandmarkIndex );
}

////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector4d& Map::LandmarkWorkRef( MeasurementId nId )
{
    SlamFramePtr pFrame = GetFramePtr( nId.m_nLandmarkId.m_nRefFrameId );
    return pFrame->LandmarkWorkRef( nId.m_nLandmarkId.m_nLandmarkIndex );
}

////////////////////////////////////////////////////////////////////////////////////////
// set the transform between two frames:
void Map::SetRelativePose( int nStartId, int nEndId, Eigen::Matrix4d& Tab ) 
{
    SlamEdgePtr pEdge = GetEdgePtr( nStartId, nEndId );
    SlamFramePtr pFrame = GetFramePtr( nStartId );
    pEdge->SetTransform( nStartId, nEndId, Tab );
}

////////////////////////////////////////////////////////////////////////////////////////
// get the transform between two frames:
bool Map::GetRelativePose( int nStartId, int nEndId, Eigen::Matrix4d& Tab ) 
{
    SlamEdgePtr pEdge = GetEdgePtr( nStartId, nEndId );
    SlamFramePtr pFrame = GetFramePtr( nStartId );
    return pEdge->GetTransform( nStartId, nEndId, Tab );
}

////////////////////////////////////////////////////////////////////////////////////////
/// Return smart pointer to frames "parent"
SlamFramePtr Map::GetParentFramePtr( unsigned int uChildFrameId ) 
{
    SlamFramePtr pChildFrame = GetFramePtr( uChildFrameId );
    if( pChildFrame->ParentEdgeId() == NO_PARENT ) {
        return SlamFramePtr( (ReferenceFrame*)NULL );
    }
    SlamEdgePtr pEdge = GetEdgePtr( pChildFrame->ParentEdgeId() );
    SlamFramePtr pParentFrame;
    pParentFrame = pEdge->EndId() == uChildFrameId ? GetFramePtr(pEdge->StartId()) : GetFramePtr(pEdge->EndId());
    return pParentFrame;
}

////////////////////////////////////////////////////////////////////////////////////////
// Return reference in TargetMsr to measurement identified by SourceMsr but in a particular frame. True if found, false otherwise.
bool Map::GetMeasurementInFrame( Measurement SourceMsr, unsigned int uFrameId, Measurement& TargetMsr ) 
{
    assert( uFrameId < m_vFrames.size() );
    std::vector<Measurement>& vMsrs = m_vFrames[uFrameId]->GetMeasurementsVectorRef();

    for( unsigned int ii = 0; ii < vMsrs.size(); ++ii ) {
        if( vMsrs[ii].m_nId.m_nLandmarkId.m_nRefFrameId == SourceMsr.m_nId.m_nLandmarkId.m_nRefFrameId
                && vMsrs[ii].m_nId.m_nLandmarkId.m_nLandmarkIndex == SourceMsr.m_nId.m_nLandmarkId.m_nLandmarkIndex ) {

            TargetMsr = vMsrs[ii];
            return true;
        }
    }	
    return false;
}

////////////////////////////////////////////////////////////////////////////////////////
// Return reference in Msr to measurement identified by a Frame Id and a local measurement index. True if found, false otherwise.
bool Map::GetMeasurementInFrame( unsigned int uFrameId, unsigned int uMsrLocalIndex, Measurement& Msr) 
{
    assert( uFrameId < m_vFrames.size() );
    std::vector<Measurement>& vMsrs = m_vFrames[uFrameId]->GetMeasurementsVectorRef();
    assert( uMsrLocalIndex < vMsrs.size() );		
    Msr = vMsrs[uMsrLocalIndex];
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////
bool Map::GetMeasurement(
        const MeasurementId& nId, //< Input:
        Measurement& Msr  //< Output:
        )
{
    return GetMeasurementInFrame( nId.m_nFrameId, nId.m_nLocalIndex, Msr );
}

////////////////////////////////////////////////////////////////////////////////////////
// Return reference in Landmark corresponding to given measurement
Landmark& Map::GetLandmarkRef( Measurement Msr )
{
    unsigned nFrameId = Msr.m_nId.m_nLandmarkId.m_nRefFrameId;
    unsigned nLocalIdx = Msr.m_nId.m_nLandmarkId.m_nLandmarkIndex;
    std::vector<Landmark>& vLandmarks = m_vFrames[nFrameId]->GetLandmarksVectorRef();
    return vLandmarks[nLocalIdx];
}

////////////////////////////////////////////////////////////////////////////////////////
// Return reference in Landmark corresponding to given measurement
Landmark& Map::GetLandmarkRef( LandmarkId Id )
{
    std::vector<Landmark>& vLandmarks = m_vFrames[Id.m_nRefFrameId]->GetLandmarksVectorRef();
    return vLandmarks[Id.m_nLandmarkIndex];
}

////////////////////////////////////////////////////////////////////////////////////////
void Map::GetPosesToDepth( std::map<unsigned int, Eigen::Matrix4d>& Poses, unsigned int nDepth )
{
    Poses.clear();

    // do BFS
    // get the first node (which will be the origin of the coordinate frame)
    SlamFramePtr  pRoot  = m_vFrames.back();

    // now search for close nodes
    std::queue<SlamFramePtr> q;
    pRoot->SetDepth(0);
    q.push(pRoot);

    // reset colors
    ResetNodes();

    SlamFramePtr pCurNode;
    Eigen::Matrix4d T;

    while( !q.empty() ) {

        pCurNode = q.front();
        q.pop();
        pCurNode->SetGrey(); 

        // Explore neighbours
        for(unsigned int ii=0; ii < pCurNode->NumNeighbors(); ++ii) {
            unsigned int uNeighborEdgeId = pCurNode->GetNeighborEdgeId(ii); 
            SlamEdgePtr pNeighborEdge   = GetEdgePtr(uNeighborEdgeId);
            unsigned int uNeighborNodeId;

            uNeighborNodeId = ( pNeighborEdge->StartId() == pCurNode->Id() ) ?
                pNeighborEdge->EndId() : pNeighborEdge->StartId();

            SlamFramePtr pNeighborNode = GetFramePtr(uNeighborNodeId);

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
            SlamEdgePtr pEdge = GetEdgePtr( uParentEdgeId );
            pEdge->GetTransform(pEdge->StartId(), pEdge->EndId(), T);
            Poses[pCurNode->Id()] = T;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////
void Map::GetAbsolutePosesToDepth( 
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
    
    SlamFramePtr pRoot  = m_vFrames[nRootId];

    // now search for close nodes
    std::queue<SlamFramePtr>    q;
    std::queue<Eigen::Matrix4d> t;
    pRoot->SetDepth(0);
    q.push(pRoot);
    t.push(Eigen::Matrix4d::Identity());

    // reset colors
    ResetNodes();

    SlamFramePtr pCurNode;
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
            SlamEdgePtr pNeighborEdge   = GetEdgePtr(uNeighborEdgeId);
            unsigned int uNeighborNodeId;

            uNeighborNodeId = ( pNeighborEdge->StartId() == pCurNode->Id() ) ?
                                pNeighborEdge->EndId() : pNeighborEdge->StartId();
            
            SlamFramePtr pNeighborNode = GetFramePtr(uNeighborNodeId);

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
void Map::ResetNodes()
{
    // reset all nodes' color to black 
    std::vector<SlamFramePtr>::iterator it;

    for(it = m_vFrames.begin(); it != m_vFrames.end(); ++it) {
        (*it)->SetWhite();
        (*it)->SetDepth(0);
    }

}

////////////////////////////////////////////////////////////////////////////////////////
// Returns the Ids of nodes inside the spatio-temporal slidding window
// using Breadth-First Search (BFS) and their absolute pose
unsigned int Map::GetNodesInsideWindowBFS(
        std::map<unsigned int, Eigen::Matrix4d>& absolutePoses,
        std::map<LandmarkId, Landmark*>&         landmarks,
        std::vector<unsigned int>&               constantPosesIds,
        unsigned int                             nDepth
        )
{

    // reset colors
    ResetNodes();

    std::vector<unsigned int>::iterator it;

    // get the first node (which will be the origin of the coordinate frame)
    SlamFramePtr  pRoot  = m_vFrames.back();//[m_vFrames.size()-1];    

    // now search for close nodes
    std::queue<SlamFramePtr> q;
    std::queue<Eigen::Matrix4d> t;
    pRoot->SetDepth(0);
    q.push(pRoot);
    t.push(Eigen::Matrix4d::Identity());

    SlamFramePtr pCurNode;
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
            SlamEdgePtr pNeighborEdge    = GetEdgePtr(uNeighborEdgeId);
            unsigned int uNeighborNodeId;

            if( pNeighborEdge->StartId() == pCurNode->Id() ) {
                uNeighborNodeId = pNeighborEdge->EndId();
            } else {
                uNeighborNodeId = pNeighborEdge->StartId();
            }
            SlamFramePtr pNeighborNode = GetFramePtr(uNeighborNodeId);

            if( pCurNode->Depth() < nDepth && 
                pNeighborNode != NULL      && 
                pNeighborNode->IsWhite() ) {

                // add node to the queue
                pNeighborNode->SetDepth( pCurNode->Depth()+1 );
                pNeighborNode->SetGrey(); 
                q.push( pNeighborNode );
                // compute cummulative transformation (transformation from root)
                pNeighborEdge->GetTransform( pCurNode->Id(), uNeighborNodeId, T);
                t.push(curT*T);
            }
        }

        // We are done with this node, add it to the map
        pCurNode->SetBlack();
        absolutePoses[pCurNode->Id()] = curT;

        //******************************************************************
        // go through measurements in the node
        std::vector<Measurement>& vMsr = pCurNode->GetMeasurementsVectorRef();
        std::map<LandmarkId, Landmark*>::iterator itLmk;
        
        for( unsigned int ii=0; ii < vMsr.size(); ++ii ) { 
            Measurement& z = vMsr[ii];
            // get measurement corresponding landmark
            Landmark& Lmk = GetLandmarkRef( z );
            itLmk = landmarks.find( Lmk.Id() );
            // check if is a new landmark
            if( itLmk == landmarks.end() ) {
                //save landmark
                const std::vector<MeasurementId>& vTrack = Lmk.GetFeatureTrackRef();
                
                if( vTrack.size() < 2 ){
                    continue;
                }
                landmarks[ Lmk.Id() ] = &Lmk;
                // save id's of the frames where the landmark was observed
                for( size_t ii = 0; ii < vTrack.size(); ii++ ){
                    unsigned int uId = vTrack[ii].m_nFrameId;
                    it = std::find( constantPosesIds.begin(), constantPosesIds.end(), uId );
                    if( it == constantPosesIds.end() ){
                        constantPosesIds.push_back(uId);
                    }
                }
            }
        }
        //******************************************************************
    }

    // remove from constantPosesIds the Ids of poses inside the window
    std::map< unsigned int, Eigen::Matrix4d >::iterator mit;
    for( mit = absolutePoses.begin(); mit != absolutePoses.end(); ++mit ) {
        it = std::find(constantPosesIds.begin(),constantPosesIds.end(), mit->first);
        if( it != constantPosesIds.end() )
            constantPosesIds.erase(it);
    }

    return pRoot->Id();
}

////////////////////////////////////////////////////////////////////////////////////////
void Map::GetNodesOutsideWindowBFS(
        const unsigned int& uRootId,
        std::vector<unsigned int>& PosesIds,
        std::map<unsigned int, Eigen::Matrix4d>& absolutePoses,
        int
        )
{
    std::vector<unsigned int>::iterator it;

    // get the first node (which will be the origin of the coordinate frame)
    SlamFramePtr   pRoot  = GetFramePtr( uRootId );

    // now search for close nodes
    std::queue<SlamFramePtr> q;
    std::queue<Eigen::Matrix4d> t;
    pRoot->SetDepth(0);
    q.push(pRoot);
    t.push(Eigen::Matrix4d::Identity());

    // reset colors
    ResetNodes();

    SlamFramePtr pCurNode;
    Eigen::Matrix4d curT;
    Eigen::Matrix4d T;

    while( !q.empty() ) {
        pCurNode = q.front();
        curT         = t.front();
        q.pop();
        t.pop();
        pCurNode->SetGrey(); 

        // Explore neighbours
        for(unsigned int ii=0; ii < pCurNode->NumNeighbors(); ++ii) {
            unsigned int uNeighborEdgeId = pCurNode->GetNeighborEdgeId(ii); 
            SlamEdgePtr pNeighborEdge   = GetEdgePtr(uNeighborEdgeId);
            unsigned int uNeighborNodeId;

            if( pNeighborEdge->StartId() == pCurNode->Id() ) {
                uNeighborNodeId = pNeighborEdge->EndId();
            } else {
                uNeighborNodeId = pNeighborEdge->StartId();
            }
            SlamFramePtr pNeighborNode = GetFramePtr(uNeighborNodeId);

            if( pNeighborNode != NULL && pNeighborNode->IsWhite()) {

                // add node to the queue
                pNeighborNode->SetDepth( pCurNode->Depth()+1 );
                pNeighborNode->SetGrey(); 
                q.push( pNeighborNode );
                // compute cummulative transformation (transformation from root)
                pNeighborEdge->GetTransform(pCurNode->Id(),uNeighborNodeId, T);
                t.push(curT*T);
            }
        }

        // We are done with this node, add it to the map
        pCurNode->SetBlack();
        it = std::find(PosesIds.begin(),PosesIds.end(),pCurNode->Id());
        if( it != PosesIds.end() ) {
            absolutePoses[pCurNode->Id()] = curT;
            PosesIds.erase(it);
        }

        if( PosesIds.empty() ) break;
    }
}


////////////////////////////////////////////////////////////////////////////////////////
void Map::UpdateRelativeNodes(  std::map<unsigned int, Eigen::Matrix4d>& absolutePoses )
{
    ResetNodes();

    std::map<unsigned int, Eigen::Matrix4d>::iterator it;
    std::map<unsigned int, Eigen::Matrix4d>::iterator itLocal;
    Eigen::Matrix4d Tab;
    bool bIsHead;

    for( it = absolutePoses.begin(); it != absolutePoses.end(); ++it ) {
        SlamFramePtr pCurNode = GetFramePtr(it->first);
        pCurNode->SetGrey();
        // Explore neighbours
        for(unsigned int ii=0; ii < pCurNode->NumNeighbors(); ++ii) {
            unsigned int uNeighborEdgeId = pCurNode->GetNeighborEdgeId(ii); 
            SlamEdgePtr pNeighborEdge   = GetEdgePtr(uNeighborEdgeId);
            unsigned int uNeighborNodeId;

            if( pNeighborEdge->StartId() == pCurNode->Id() ) {
                uNeighborNodeId = pNeighborEdge->EndId();
                bIsHead = false;
            } else {
                uNeighborNodeId = pNeighborEdge->StartId();
                bIsHead = true;
            }

            itLocal = absolutePoses.find(uNeighborNodeId);

            // only update relative edges between optimized poses
            if(itLocal == absolutePoses.end()) continue;

            SlamFramePtr pNeighborNode = GetFramePtr(uNeighborNodeId);

            if( pNeighborNode != NULL && pNeighborNode->IsWhite() ) {
                // compute edge relative transformation
                if ( bIsHead ) {
                    //std::cout << "a: " << uNeighborNodeId << " b:" << it->first << "  ";
                    Tab = itLocal->second * it->second.inverse();
                } else {
                    //std::cout << "a: " << it->first  << " b:" << uNeighborNodeId << "  ";
                    Tab = it->second * itLocal->second.inverse(); 
                }
                mvl::MakeOrthonormal(Tab);
                //std::cout << "Tab:" << std::endl << Tab << std::endl;
                pNeighborEdge->SetTransform(Tab);
                pNeighborNode->SetGrey();           
            }
        }// end of neighbor exploration
    }// end of main cycle
}

////////////////////////////////////////////////////////////////////////////////////////
void Map::Print()
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
        printf("Id: %d  num landmarks: %d  num measurements: %d \n", 
        m_vFrames[ii]->Id(), m_vFrames[ii]->NumLandmarks(), m_vFrames[ii]->NumMeasurements());
    }
}
