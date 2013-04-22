#include <queue>

#include <Mvlpp/Mvl.h>
#include <RPG/Utils/ImageWrapper.h>

#include "DenseMap.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
DenseMap::DenseMap()
{
    m_dLastModifiedTime = 0;
    m_bFitPlane         = true;
    m_dPathOrientation.setIdentity();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
DenseMap::~DenseMap()
{
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DenseMap::LoadCameraModels(
        const std::string&          GreyCModFile,       //< Input: Grey camera model file name
        const std::string&          DepthCModFile       //< Input: Depth camera model file name
    )
{
    // get intrinsics
    if( !m_CModPyrGrey.Read( GreyCModFile ) ) {
        return false;
    }

    if( !m_CModPyrDepth.Read( DepthCModFile ) ) {
        return false;
    }

    return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
FramePtr DenseMap::NewFrame(
        double          dTime,                  //< Input: Sensor time
        const cv::Mat&  GreyImage,              //< Input: Greyscale image
        const cv::Mat&  GreyThumb               //< Input: Greyscale thumbnail
    )
{
    FramePtr pFrame( new ReferenceFrame );
    pFrame->SetId( m_vFrames.size() );
    pFrame->SetTime( dTime );
    pFrame->SetGreyImage( GreyImage );
    pFrame->SetGreyThumb( GreyThumb );
    Lock();
    _UpdateModifiedTime();
    m_vFrames.push_back( pFrame );
    Unlock();
    return pFrame;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
FramePtr DenseMap::NewKeyframe(
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
    pFrame->SetKeyframeFlag();
    pFrame->SetGreyImage( GreyImage );
    pFrame->SetDepthImage( DepthImage );
    pFrame->SetGreyThumb( GreyThumb );
    pFrame->SetDepthThumb( DepthThumb );
    Lock();
    _UpdateModifiedTime();
    m_vFrames.push_back( pFrame );
    Unlock();
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
    return false;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DenseMap::LinkFrames(
        FramePtr                    pA,         //< Input:
        FramePtr                    pB,         //< Input:
        const Eigen::Matrix4d&      Tab         //< Input:
    )
{
    assert( pA->GetId() < m_vFrames.size() );
    assert( pB->GetId() < m_vFrames.size() );
    assert( pA->GetId() == m_vFrames[pA->GetId()]->GetId() );
    assert( pB->GetId() == m_vFrames[pB->GetId()]->GetId() );

    if( EdgeExists( pA->GetId(), pB->GetId() ) ) {
        std::cerr << "warning: trying to add an edge that already exists!" << std::endl;
        return false;
    }

    EdgePtr pEdge( new TransformEdge( pA->GetId(), pB->GetId(), Tab ) );
    unsigned int nEdgeId = m_vEdges.size();

    pA->AddNeighbor( nEdgeId );
    pB->AddNeighbor( nEdgeId );
    pB->SetParentEdgeId( nEdgeId );
    Lock();
    _UpdateModifiedTime();
    m_vEdges.push_back( pEdge );
    Unlock();

    return true;
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
        unsigned int    nGetStartId,    //< Input
        unsigned int    nEndId          //< Input
    )
{
    unsigned int tmp;
    return FindEdgeId( nGetStartId, nEndId, tmp );
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DenseMap::IsKeyframe(
        unsigned int    nFrameId        //< Input
    )
{
    if( nFrameId >= m_vFrames.size() ) {
        return false;
    }
    return m_vFrames[nFrameId]->IsKeyframe();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3d DenseMap::GetGreyCameraK(
        unsigned int    nLevel          //< Input: Pyramid level intrinsic
    )
{
    if( m_CModPyrGrey.IsInit() == false ) {
        std::cerr << "abort: Camera model files have not been initialized but are being requested. Was LoadCameraModels called?" << std::endl;
        exit(1);
    }
    return m_CModPyrGrey.K( nLevel );
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3d DenseMap::GetDepthCameraK(
        unsigned int    nLevel          //< Input: Pyramid level intrinsic
    )
{
    if( m_CModPyrGrey.IsInit() == false ) {
        std::cerr << "abort: Camera model files have not been initialized but are being requested. Was LoadCameraModels called?" << std::endl;
        exit(1);
    }
    return m_CModPyrDepth.K( nLevel );
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4d DenseMap::GetGreyCameraPose()
{
    return m_CModPyrGrey.GetPose();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4d DenseMap::GetDepthCameraPose()
{
    return m_CModPyrDepth.GetPose();
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
//        return false; // nothing to do, map is up-to date
    }

    if( m_CModPyrGrey.IsInit() == false ) {
        m_CModPyrGrey.Read( rRHS.m_CModPyrGrey.GetCamModFilename() );
    }

    if( m_CModPyrDepth.IsInit() == false ) {
        m_CModPyrDepth.Read( rRHS.m_CModPyrDepth.GetCamModFilename() );
    }

    // make sure we're dealing with the same map?
//        assert( m_nMapId == rRHS.m_nMapId );

    // find data we need to update.  All we need to do here is search in the SLAM graph until
    // we stop finding nodes (Landmarks or Frames) that have been modified since m_dLastModifiedTime
//        _DFSCopy( )
    // HACK TODO FIXME  -- just to get going, copy the last 100 frames as a quick hack
    m_vEdges.resize( rRHS.m_vEdges.size() );
    m_vFrames.resize( rRHS.m_vFrames.size() );

    for( int ii = rRHS.m_vEdges.size()-1; ii >= std::max( (int)rRHS.m_vEdges.size()-500, 0 ); ii-- ) {
        m_vEdges[ii] = std::shared_ptr<TransformEdge>( new TransformEdge( *rRHS.m_vEdges[ii] ) );
        //m_vEdges[ii] = rRHS.m_vEdges[ii]; // will do a deep copy
    }

    for( int ii = rRHS.m_vFrames.size()-1; ii >= std::max( (int)rRHS.m_vFrames.size()-500, 0 ); ii-- ) {
        m_vFrames[ii] = std::shared_ptr<ReferenceFrame>( new ReferenceFrame( *rRHS.m_vFrames[ii] ) );
    }

    m_pCurKeyframe = std::shared_ptr<ReferenceFrame>( new ReferenceFrame( *(rRHS.m_pCurKeyframe) ) );

    // copy internal path and orientation
    m_dPathOrientation = rRHS.m_dPathOrientation;
    m_vPath.clear();
    for( unsigned int ii = 0; ii < rRHS.m_vPath.size(); ++ii ) {
        m_vPath[ii] = rRHS.m_vPath[ii];
    }

    // TODO add a mutex around this whole call, and use it also in _UpdateModifiedTime()
    // this way modifications of the map during copy are still preserved by the ModifiedTime discrepancy??
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
bool DenseMap::SetKeyframe(
        unsigned int    nKeyframeId       //< Input: Keyframe ID
    )
{
    if( nKeyframeId >= m_vFrames.size() ) {
        return false;
    }

    FramePtr pKeyframe = GetFramePtr( nKeyframeId );
    if( pKeyframe->IsKeyframe() == false ) {
        return false;
    }

    m_pCurKeyframe = pKeyframe;
    return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DenseMap::FindClosestKeyframes(
        const Eigen::Matrix4d&                              dPose,          //< Input
        float                                               fMaxNorm,       //< Input
        std::vector< std::pair< unsigned int, float > >&    vKeyframes      //< Output
    )
{

    Eigen::Matrix4d dInvPose = mvl::TInv( dPose );
    for( unsigned int ii = 0; ii < m_vPath.size(); ++ii ) {
        if( IsKeyframe(ii) ) {
            Eigen::Matrix4d PoseError = dInvPose * m_vPath[ii];
            Eigen::Vector6d deltaPose = mvl::T2Cart( PoseError );
            float fNorm = deltaPose.norm();
            if( fNorm < fMaxNorm ) {
                vKeyframes.push_back( std::pair<unsigned int, float>( ii, fNorm ) );
            }
        }
    }

    // sort vector
    std::sort( vKeyframes.begin(), vKeyframes.end(), _CompareNorm );
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DenseMap::PrintMap()
{
    printf("***************************************************************************************************************************\n");
    printf("\tPRINT MAP\n");
    printf("***************************************************************************************************************************\n");
    for( unsigned int ii = 0; ii < m_vFrames.size(); ++ii ) {
        FramePtr pFrame = m_vFrames[ii];
        assert( ii == pFrame->GetId() );
        Eigen::Vector6d E = mvl::T2Cart( m_vPath[ii] );
        printf( "Frame ID: %05d \t Keyframe: %d \t Current Global Pose: [ %+.2f, %+.2f, %+.2f, %+.2f, %+.2f, %+.2f ]\n", ii,
                pFrame->IsKeyframe(), E(0), E(1), E(2), E(3), E(4), E(5) );

        std::vector<unsigned int>& vNeighbors = pFrame->GetNeighbors();
        for( unsigned int& ii : vNeighbors ) {
            EdgePtr pEdge = GetEdgePtr( ii );
            const unsigned int nStartId = pEdge->GetStartId();
            const unsigned int nEndId = pEdge->GetEndId();
            Eigen::Matrix4d Tse;
            pEdge->GetOriginalTransform( nStartId, nEndId, Tse );
            E = mvl::T2Cart( Tse );
            printf( "---  Start: %05d \t End: %05d \t Relative Transform: [ %+.2f, %+.2f, %+.2f, %+.2f, %+.2f, %+.2f ]\n", nStartId, nEndId,
                    E(0), E(1), E(2), E(3), E(4), E(5) );
        }
        printf("-----------------------------------------------------------------------------------------------------------------\n");
    }
    printf( "Total Frames: %5d \t\t Total Edges: %5d\n\n", GetNumFrames(), GetNumEdges() );
    fflush(stdout);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DenseMap::ExportMap()
{
    printf("-----------------------------------------------------------------------------------------------------------------\n");
    printf("Exporting map...\n");
    fflush(stdout);

    const std::string sFilename = "Map.xml";
    cv::FileStorage fs( sFilename, cv::FileStorage::WRITE );

    fs << "Frames" << "[";
    for( unsigned int ii = 0; ii < m_vFrames.size(); ++ii ) {
        FramePtr pFrame = m_vFrames[ii];
        assert( ii == pFrame->GetId() );

        // save images
        /*
        char            Index[10];
        sprintf( Index, "%08d", ii );

        std::string FileName;
        std::string GreyPrefix = "Export/Grey";
        cv::Mat GreyImage = pFrame->GetGreyImageRef();
        FileName = GreyPrefix + "-I-" + Index + ".pgm";
        cv::imwrite( FileName, GreyImage );
        cv::Mat GreyThumb = pFrame->GetGreyThumbRef();
        FileName = GreyPrefix + "-T-" + Index + ".pgm";
        cv::imwrite( FileName, GreyThumb );
        if( pFrame->IsKeyframe() ) {
            std::string DepthPrefix = "Export/Depth";
            rpg::ImageWrapper DepthImage;
            FileName = DepthPrefix + "-I-" + Index + ".pdm";
            DepthImage.Image = pFrame->GetDepthImageRef();
            DepthImage.write( FileName, false );
            rpg::ImageWrapper DepthThumb;
            FileName = DepthPrefix + "-T-" + Index + ".pdm";
            DepthThumb.Image = pFrame->GetDepthThumbRef();
            DepthThumb.write( FileName, false );
        }
        */

        // save info
        fs << "{:";
        fs << "FrameId" << (int)ii;
        fs << "ParentId" << (int)pFrame->GetParentEdgeId();
        fs << "Time" << (double)pFrame->GetTime();
        fs << "Keyframe" << pFrame->IsKeyframe();
        cv::Mat& GreyImage = pFrame->GetGreyImageRef();
        fs << "GreyImage" << GreyImage;
        cv::Mat& GreyThumb = pFrame->GetGreyThumbRef();
        fs << "GreyThumb" << GreyThumb;
        if( pFrame->IsKeyframe() ) {
            cv::Mat& DepthImage = pFrame->GetDepthImageRef();
            fs << "DepthImage" << DepthImage;
            cv::Mat& DepthThumb = pFrame->GetDepthThumbRef();
            fs << "DepthThumb" << DepthThumb;
        }
        fs << "}";
    }
    fs << "]";


    fs << "Edges" << "[";
    for( unsigned int ii = 0; ii < m_vEdges.size(); ++ii ) {
        EdgePtr pEdge = m_vEdges[ ii ];
        const unsigned int nStartId = pEdge->GetStartId();
        const unsigned int nEndId = pEdge->GetEndId();
        // TODO export also original transform
        Eigen::Matrix4d Tse;
        pEdge->GetTransform( nStartId, nEndId, Tse );
        Eigen::Vector6d E = mvl::T2Cart( Tse );
        fs << "{:";
        fs << "StartId" << (int)nStartId;
        fs << "EndId" << (int)nEndId;
        fs << "Tse" << "[:" << E(0) << E(1) << E(2) << E(3) << E(4) << E(5);
        fs << "]";
        fs << "}";
    }
    fs << "]";


    fs.release();
    printf( "Total Frames: %5d \t\t Total Edges: %5d\n", GetNumFrames(), GetNumEdges() );
    printf("... Done!!\n");
    printf("-----------------------------------------------------------------------------------------------------------------\n");
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DenseMap::ImportMap()
{
    printf("-----------------------------------------------------------------------------------------------------------------\n");
    printf("Importing map...\n");
    fflush(stdout);

    const std::string sFilename = "Map.xml";
    cv::FileStorage fs( sFilename, cv::FileStorage::READ );

    cv::FileNode nodeFrames = fs["Frames"];
    if( nodeFrames.type() != cv::FileNode::SEQ ) {
        std::cerr << "Error reading the map. Aborting!" << std::endl;
        return;
    }

    cv::FileNodeIterator itFrame = nodeFrames.begin(), itFrame_end = nodeFrames.end();
    for( ; itFrame != itFrame_end; ++itFrame ) {
        int nFrameId;
        (*itFrame)["FrameId"] >> nFrameId;
        int nParentId;
        (*itFrame)["ParentId"] >> nParentId;
        double dTime;
        (*itFrame)["Time"] >> dTime;
        bool bIsKeyframe;
        (*itFrame)["Keyframe"] >> bIsKeyframe;
        cv::Mat GreyImage, GreyThumb;
        (*itFrame)["GreyImage"] >> GreyImage;
        (*itFrame)["GreyThumb"] >> GreyThumb;
        if( bIsKeyframe ) {
            cv::Mat DepthImage, DepthThumb;
            (*itFrame)["DepthImage"] >> DepthImage;
            (*itFrame)["DepthThumb"] >> DepthThumb;
            FramePtr pFrame = NewKeyframe( dTime, GreyImage, DepthImage, GreyThumb, DepthThumb );
            if( nFrameId != (int)pFrame->GetId() ) {
                std::cerr << "error: FrameId does not match ID of import file!" << std::endl;
                return;
            }
        } else {
            FramePtr pFrame = NewFrame( dTime, GreyImage, GreyThumb );
            if( nFrameId != (int)pFrame->GetId() ) {
                std::cerr << "error: FrameId does not match ID of import file!" << std::endl;
                return;
            }
        }
    }

    cv::FileNode nodeEdges = fs["Edges"];
    if( nodeEdges.type() != cv::FileNode::SEQ ) {
        std::cerr << "Error reading the map. Aborting!" << std::endl;
        return;
    }

    cv::FileNodeIterator itEdge = nodeEdges.begin(), itEdge_end = nodeEdges.end();
    for( ; itEdge != itEdge_end; ++itEdge ) {
        int nStartId;
        (*itEdge)["StartId"] >> nStartId;
        int nEndId;
        (*itEdge)["EndId"] >> nEndId;
        Eigen::Vector6d Tse;
        cv::FileNode nodePose = (*itEdge)["Tse"];
        cv::FileNodeIterator itPose = nodePose.begin(), itPose_end = nodePose.end();
        unsigned int idx = 0;
        for( ; itPose != itPose_end; ++itPose ) {
            (*itPose) >> Tse(idx);
            idx++;
        }
        FramePtr pA = GetFramePtr( nStartId );
        FramePtr pB = GetFramePtr( nEndId );
        LinkFrames( pA, pB, mvl::Cart2T(Tse) );
    }

    fs.release();
    SetKeyframe(0);

    printf( "Total Frames: %5d \t\t Total Edges: %5d\n", GetNumFrames(), GetNumEdges() );
    printf("... Done!!\n");
    printf("-----------------------------------------------------------------------------------------------------------------\n");
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
            unsigned int nNeighborEdgeId = pCurNode->GetNeighborEdgeId(ii);
            EdgePtr pNeighborEdge = GetEdgePtr(nNeighborEdgeId);

            unsigned int nNeighborNodeId;
            nNeighborNodeId = ( pNeighborEdge->GetStartId() == pCurNode->GetId() ) ?
                                pNeighborEdge->GetEndId() : pNeighborEdge->GetStartId();

            FramePtr pNeighborNode = GetFramePtr(nNeighborNodeId);

            if( (int)pCurNode->GetDepth() < nDepth &&
                pNeighborNode != NULL      &&
                pNeighborNode->IsWhite() ) {

                // add node to the queue
                pNeighborNode->SetDepth( pCurNode->GetDepth()+1 );
                pNeighborNode->SetGrey();
                q.push( pNeighborNode );

                // compute cummulative transformation (transformation from root)
                pNeighborEdge->GetTransform( pCurNode->GetId(), nNeighborNodeId, T );
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
void DenseMap::UpdateInternalPath()
{
    Lock();
    FramePtr pLastFrameInMap = m_vFrames.back();
    unsigned int nLastFrameInMap = pLastFrameInMap->GetId();
    unsigned int nLastFrameInPath = m_vPath.rbegin()->first;

    std::map< unsigned int, Eigen::Matrix4d >   vNearPath;
    GenerateAbsolutePoses( vNearPath, -1, 3 );
    auto it = vNearPath.find( nLastFrameInPath );
    if( it != vNearPath.end() ) {
        const Eigen::Matrix4d Twl = it->second;
        // since the parent of this new current frame was the previous "world origin"
        // we simply add the new frame as origin, and multiply the Twc-1 to bring the path
        // to the current's frame reference frame
        for( unsigned int ii = 0; ii < m_vPath.size(); ++ii ) {
            m_vPath[ii] = Twl * m_vPath[ii];
        }
        m_vPath[nLastFrameInMap] = Eigen::Matrix4d::Identity();
    } else {
        GenerateAbsolutePoses( m_vPath );
    }
//    _DynamicGroundPlaneEstimation();
    _UpdateModifiedTime();
    Unlock();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DenseMap::UpdateInternalPathFull()
{
    Lock();
    GenerateAbsolutePoses( m_vPath );
    _DynamicGroundPlaneEstimation();
    _UpdateModifiedTime();
    Unlock();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::map< unsigned int, Eigen::Matrix4d >& DenseMap::GetInternalPath()
{
    return m_vPath;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4d& DenseMap::GetPathOrientation()
{
    return m_dPathOrientation;
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DenseMap::_DynamicGroundPlaneEstimation() {

    if( m_vPath.size() < 50  || m_bFitPlane == false ) {
        return;
    }

    Eigen::Matrix4d dOrigin = mvl::TInv( m_vPath[0] );

    std::vector< Eigen::Vector3d > vPoints;
    for( unsigned int ii = 0; ii < m_vPath.size(); ++ii ) {
        Eigen::Matrix4d Pose4 = dOrigin * m_vPath[ii];
        Eigen::Vector3d Pose3 = Pose4.block<3,1>(0,3);
        vPoints.push_back( Pose3 );
    }

    double distance = 0.0;

    // compute mean
    Eigen::Vector3d mean(0.0, 0.0, 0.0);
    mean += vPoints[0];
    for( unsigned int ii = 1; ii < vPoints.size(); ++ii ) {
        mean += vPoints[ii];
        Eigen::Vector3d diff = vPoints[ii] - vPoints[ii-1];
        distance += diff.norm();
    }
    mean /= vPoints.size();

    // compute covariance matrix
    Eigen::Matrix3d cov;
    cov.setZero();
    for( unsigned int ii = 0; ii < vPoints.size(); ++ii ) {
        Eigen::Vector3d p = vPoints[ii] - mean;
        cov(0,0) += p(0)*p(0);
        cov(1,1) += p(1)*p(1);
        cov(2,2) += p(2)*p(2);
        cov(0,1) += p(0)*p(1);
        cov(0,2) += p(0)*p(2);
        cov(1,2) += p(1)*p(2);
    }
    cov(1,0) = cov(0,1);
    cov(2,0) = cov(0,2);
    cov(2,1) = cov(1,2);
    cov /= (vPoints.size() - 1);

    // compute principal components using svd
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Vector3d S = svd.singularValues();

    double threshold = 1e-4;
    if( S(0) < threshold || S(1) < threshold ) {
        return;
    }

    if( V.determinant() < 0.0 ) {
        return;
    }

    // the normal of the plane is the singular vector with the smallest
    // singular value (direction of least variance)
    Eigen::Vector3d down = V.block<3,1>(0,2).normalized();

    // check that vector is pointing down
    Eigen::Vector3d z( 0.0, 0.0, 1.0 );
    if( down.dot( z ) < 0.0 ) {
        return;
    }

    // compute path transformation
    Eigen::Vector3d forward( 1.0, 0.0, 0.0 );
    Eigen::Vector3d right = down.cross( forward );
    right.normalize();
    forward = right.cross( down );
    forward.normalize();

    m_dPathOrientation.block<1,3>(0,0) = forward;
    m_dPathOrientation.block<1,3>(1,0) = right;
    m_dPathOrientation.block<1,3>(2,0) = down;

    // if we travelled more than 50 m stop fitting the plane
    if( distance > 50.0 ) {
        m_bFitPlane = false;
    }

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DenseMap::_CompareNorm(
        std::pair< unsigned int, float > lhs,
        std::pair< unsigned int, float > rhs
    )
{
    return std::get<1>(lhs) < std::get<1>(rhs);
}
