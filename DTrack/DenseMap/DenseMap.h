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

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    DenseMap();

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ~DenseMap();

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // allocate a new frame, but do not link it into the graph
    FramePtr NewFrame(
            double          dTime,                  //< Input: Sensor time
            const cv::Mat&  GreyImage,              //< Input: Greyscale image
            const cv::Mat&  GreyThumb               //< Input: Greyscale thumbnail
        );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // allocate a new frame, but do not link it into the graph
    FramePtr NewKeyframe(
            double              dTime,              //< Input: Sensor time
            const cv::Mat&      GreyImage,          //< Input: Greyscale image
            const cv::Mat&      DepthImage,         //< Input: Depth image
            const cv::Mat&      GreyThumb,          //< Input: Greyscale thumbnail
            const cv::Mat&      DepthThumb          //< Input: Depth thumbnail
        );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // creates a new edge with the provided transform linking frame A to frame B
    void LinkFrames(
            FramePtr                    pA,         //< Input:
            FramePtr                    pB,         //< Input:
            const Eigen::Matrix4d&      Tab         //< Input:
        );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // lookup the id of edge between start and end frame (if it exists)
    bool FindEdgeId(
            unsigned int        nStartId,       //< Input:
            unsigned int        nEndId,         //< Input:
            unsigned int&       nEdgeId         //< Output: Global ID of edge
        );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // true if ID of frame exists
    bool FrameExists(
            unsigned int    nId     //< Input:
        );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // true if there is an edge between start and end frame
    bool EdgeExists(
            unsigned int    nStartId,       //< Input:
            unsigned int    nEndId          //< Input:
        );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    FramePtr GetFirstFramePtr( );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get pointer of edge given ID
    EdgePtr GetEdgePtr( unsigned int nEdgeId );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get pointer of edge between start and end frame
    EdgePtr GetEdgePtr( unsigned int nStartId, unsigned int nEndId );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get pointer of frame given ID
    FramePtr GetFramePtr( unsigned int nFrameId );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    unsigned int GetNumFrames() { return m_vFrames.size(); }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    unsigned int GetNumEdges() { return m_vEdges.size(); }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    double GetLastModifiedTime() { return m_dLastModifiedTime; }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // set the transform between two frames
    void SetRelativeTransform( int nStartId, int nEndId, Eigen::Matrix4d& Tab );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get the transform between two frames
    bool GetRelativeTransform( int nStartId, int nEndId, Eigen::Matrix4d& Tab );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool GetTransformFromParent(
            unsigned int        nChildFrameId,  //< Input: Child ID we will find parent of
            Eigen::Matrix4d&    dTab            //< Output: Found transform from parent to child
        );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // return smart pointer to frames "parent"
    FramePtr GetParentFramePtr( unsigned int nChildFrameId );




    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // traverse the graph via BFS up to input depth and store relative poses relative to input root
    void GenerateRelativePoses(
            std::map<unsigned int, Eigen::Matrix4d>&    vPoses,         //< Output: Poses id and relative poses from input root
            int                                         nRootId = -1,   //< Input: Frame that will be the origin [default: last frame]
            int                                         nDepth = -1     //< Input: Grapth depth search from root [default: max depth]
        );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // traverse the graph via BFS up to input depth and store absolute poses relative to input root
    void GenerateAbsolutePoses(
            std::map<unsigned int, Eigen::Matrix4d>&    vPoses,         //< Output: Poses id and absolute poses from input root
            int                                         nRootId = -1,   //< Input: Frame that will be the origin [default: last frame]
            int                                         nDepth = -1     //< Input: Grapth depth search from root [default: max depth]
        );







    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    std::vector<EdgePtr>& GetEdges() { return m_vEdges; }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // set frame as current keyframe
    void SetKeyframe(
            FramePtr    pKeyframe       //< Input: Pointer to keyframe
        );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get pointer to current keyframe
    FramePtr GetCurrentKeyframe();

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Copy changes from RHS into this
    // TODO for large copies, this could break the front end update speed.  Fix this by making
    // the copy process a separate thread and having it only copy for a fixed amount of time
    // until giving up control to the front end.
    bool CopyMapChanges(
            DenseMap&       rRHS        //< Input: Map to copy from
        );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void Print();


private:

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // updates the map's modified time -- to be called whenever a change occurs in the map
    void _UpdateModifiedTime();

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // reset color and depth of all of the map's frames
    void _ResetNodes();


/////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:

    FramePtr                                    m_pCurKeyframe;
    double                                      m_dLastModifiedTime;    // crucial that we keep this up-to date

    std::vector< FramePtr >                     m_vFrames;              // list of map's reference frames
    std::vector< EdgePtr >                      m_vEdges;               // list of map's edges

};

#endif
