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
            unsigned int        uStartId,       //< Input:
            unsigned int        uEndId,         //< Input:
            unsigned int&       uEdgeId         //< Output: Global ID of edge
        );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // true if ID of frame exists
    bool FrameExists(
            unsigned int    uId     //< Input:
        );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // true if there is an edge between start and end frame
    bool EdgeExists(
            unsigned int    uStartId,       //< Input:
            unsigned int    uEndId          //< Input:
        );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    FramePtr GetFirstFramePtr( );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get pointer of edge given ID
    EdgePtr GetEdgePtr( unsigned int uEdgeId );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get pointer of edge between start and end frame
    EdgePtr GetEdgePtr( unsigned int uStartId, unsigned int uEndId );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get pointer of frame given ID
    FramePtr GetFramePtr( unsigned int uFrameId );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    unsigned int NumFrames() { return m_vFrames.size(); }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    unsigned int NumEdges() { return m_vEdges.size(); }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // set the transform between two frames
    void SetRelativeTransform( int nStartId, int nEndId, Eigen::Matrix4d& Tab );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get the transform between two frames
    bool GetRelativeTransform( int nStartId, int nEndId, Eigen::Matrix4d& Tab );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool GetTransformFromParent(
            unsigned int        uChildFrameId,  //< Input: Child ID we will find parent of
            Eigen::Matrix4d&    dTab            //< Output: Found transform from parent to child
        );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // return smart pointer to frames "parent"
    FramePtr GetParentFramePtr( unsigned int uChildFrameId );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    std::vector<EdgePtr>& GetEdges() { return m_vEdges; }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // adds path's base (global) pose -- normally identity if no prior map is provided
    void SetPathBasePose(
            const Eigen::Matrix4d&      Pose        //< Input: Base pose of path
        );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // add new estimated relative transform to path list
    void AddPathTransform(
            const Eigen::Matrix4d&      Tab         //< Input: Relative transform
        );

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


/////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
    double                              m_dLastModifiedTime;        // crucial that we keep this up-to date

    std::vector< FramePtr >             m_vFrames;                  // list of map's reference frames
    std::vector< EdgePtr >              m_vEdges;                   // list of map's edges

    // this perhaps shouldn't be here, since it is not really part of the map
    // but since the GUI already has a pointer to the map, it is easier for visualization
    FramePtr                            m_pCurKeyframe;
    Eigen::Matrix4d                     m_dBasePose;                // base pose from which path starts
    std::vector < Eigen::Matrix4d >     m_vPath;                    // vector of poses being estimated

};

#endif
