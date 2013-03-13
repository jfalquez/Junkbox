#ifndef _DENSE_MAP_H_
#define _DENSE_MAP_H_

#include <map>
#include <boost/shared_ptr.hpp>

#include "ReferenceFrame.h"
#include "TransformEdge.h"

typedef boost::shared_ptr< ReferenceFrame >  FramePtr;
typedef boost::shared_ptr< TransformEdge >   EdgePtr;

class DenseMap
{
public:

    ////////////////////////////////////////////////////////////////////////////////////////
    /// Allocate a new frame, but do not link it into the graph.
    FramePtr NewFrame( double dTime );

    ////////////////////////////////////////////////////////////////////////////////////////
    void LinkFrames( FramePtr pA, FramePtr pB, Eigen::Matrix4d& Tab );

    ////////////////////////////////////////////////////////////////////////////////////////
    // lookup the id
    bool FindEdgeId( unsigned int  uStartId,
                     unsigned int  uEndId,
                     unsigned int& uEdgeId );  //< Output: global Id of edge

    ////////////////////////////////////////////////////////////////////////////////////////
    bool HasFrame( unsigned int uId );

    ////////////////////////////////////////////////////////////////////////////////////////
    bool HasEdge( unsigned int uStartId, unsigned int uEndId );

    ////////////////////////////////////////////////////////////////////////////////////////
    FramePtr GetFirstFramePtr( );

    ////////////////////////////////////////////////////////////////////////////////////////
    EdgePtr GetEdgePtr( unsigned int uStartId, unsigned int uEndId );

    ////////////////////////////////////////////////////////////////////////////////////////
    EdgePtr GetEdgePtr( unsigned int uEdgeId );

    ////////////////////////////////////////////////////////////////////////////////////////
    FramePtr GetFramePtr( unsigned int uFrameId );

    ////////////////////////////////////////////////////////////////////////////////////////
    unsigned int NumFrames() { return m_vFrames.size(); }

    unsigned int NumEdges() { return m_vEdges.size(); }


    ////////////////////////////////////////////////////////////////////////////////////////
    bool GetTransformFromParent(  unsigned int uChildFrameId,   //< Input: Child ID we will find parent of
                                  Eigen::Matrix4d& dTab );      //< Output: Found transform from parent to child

    ////////////////////////////////////////////////////////////////////////////////////////
    // set the transform between two frames:
    void SetRelativePose( int nStartId, int nEndId, Eigen::Matrix4d& Tab );

    ////////////////////////////////////////////////////////////////////////////////////////
    // get the transform between two frames:
    bool GetRelativePose( int nStartId, int nEndId, Eigen::Matrix4d& Tab );

    ////////////////////////////////////////////////////////////////////////////////////////
    /// Return smart pointer to frames "parent"
    FramePtr GetParentFramePtr( unsigned int uChildFrameId );

    ////////////////////////////////////////////////////////////////////////////////////////
    std::vector<EdgePtr>& GetEdges() {return m_vEdges;}

    ////////////////////////////////////////////////////////////////////////////////////////
    void GetPosesToDepth(
         std::map<unsigned int, Eigen::Matrix4d>& Poses,
         unsigned int nDepth = 10
        );

    ////////////////////////////////////////////////////////////////////////////////////////
    void GetAbsolutePosesToDepth(
         std::map<unsigned int, Eigen::Matrix4d>& Poses,       //<Input: Poses id and absolute transf. from origin
         unsigned int                             nRootId,     //<Input: Node that will be the origin. [default: first pose]
         int                                      nDepth = -1  //<Input: Grapth depth search from root [default: all nodes]
        );

    ////////////////////////////////////////////////////////////////////////////////////////
    void ResetNodes();

    ////////////////////////////////////////////////////////////////////////////////////////
    void UpdateRelativeNodes(  std::map<unsigned int, Eigen::Matrix4d>& absolutePoses );

    ////////////////////////////////////////////////////////////////////////////////////////
    /// Copy changes from RHS into this
    // TODO for large copies, this could break the front end update speed.  Fix this by making
    // the copy process a separate thread and having it only copy for a fixed amount of time
    // until giving up control to the front end.
    void CopyMapChanges( DenseMap& rRHS )
    {
        //if( m_dLastModifiedTime >= rRHS.m_dLastModifiedTime ){
        //    return; // nothing to do, map is up-to date
        //}

        // make sure we're dealing with the same map?
//        assert( m_nMapId == rRHS.m_nMapId );

        // find data we need to update.  All we need to do here is search in the SLAM graph until
        // we stop finding nodes (Landmarks or Frames) that have been modified since m_dLastModifiedTime
//        _DFSCopy( )
        // HACK TODO FIXME  -- just to get going, copy the last 100 frames as a quick hack
        m_vEdges.resize( rRHS.m_vEdges.size() );
        m_vFrames.resize( rRHS.m_vFrames.size() );

        for( int ii = rRHS.m_vEdges.size()-1; ii >= std::max( (int)rRHS.m_vEdges.size()-100, 0 ); ii-- ){
            m_vEdges[ii] = boost::shared_ptr<TransformEdge>( new TransformEdge( *rRHS.m_vEdges[ii] ) );
            //m_vEdges[ii] = rRHS.m_vEdges[ii]; // will do a deep copy
        }

        for( int ii = rRHS.m_vFrames.size()-1; ii >= std::max( (int)rRHS.m_vFrames.size()-100, 0 ); ii-- ){
            m_vFrames[ii] = boost::shared_ptr<ReferenceFrame>( new ReferenceFrame( *rRHS.m_vFrames[ii] ) );
            //m_vFrames[ii] = rRHS.m_vFrames[ii]; // will do a deep copy
        }

        m_dLastModifiedTime = rRHS.m_dLastModifiedTime;
    }

    ////////////////////////////////////////////////////////////////////////////////////////
    void Print();

private:

    double                       m_dLastModifiedTime; // crucial that we keep this up-to date

    std::vector<FramePtr>    m_vFrames;
    std::vector<EdgePtr>     m_vEdges;

};

#endif
