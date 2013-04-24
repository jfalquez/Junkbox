#ifndef _DENSE_MAP_H_
#define _DENSE_MAP_H_

#include <map>
#include <mutex>
#include <memory>

#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#include <sophus/sophus.hpp>
#pragma GCC diagnostic pop

#include <Utils/CamModelPyramid.h>


#include "ReferenceFrame.h"
#include "TransformEdge.h"

typedef std::shared_ptr< ReferenceFrame >  FramePtr;
typedef std::shared_ptr< TransformEdge >   EdgePtr;

class DenseMap
{
public:

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    DenseMap();


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ~DenseMap();


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// load camera model files into a camera model pyramid
    bool LoadCameraModels(
            const std::string&          GreyCModFile,       //< Input: Grey camera model file name
            const std::string&          DepthCModFile       //< Input: Depth camera model file name
        );


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// allocate a new frame, but do not link it into the graph
    FramePtr NewFrame(
            double          dTime,                  //< Input: Sensor time
            const cv::Mat&  GreyImage,              //< Input: Greyscale image
            const cv::Mat&  GreyThumb               //< Input: Greyscale thumbnail
        );


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// allocate a new frame, but do not link it into the graph
    FramePtr NewKeyframe(
            double              dTime,              //< Input: Sensor time
            const cv::Mat&      GreyImage,          //< Input: Greyscale image
            const cv::Mat&      DepthImage,         //< Input: Depth image
            const cv::Mat&      GreyThumb,          //< Input: Greyscale thumbnail
            const cv::Mat&      DepthThumb          //< Input: Depth thumbnail
        );


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// creates a new edge with the provided transform linking frame A to frame B
    bool LinkFrames(
            FramePtr                    pA,         //< Input:
            FramePtr                    pB,         //< Input:
            const Eigen::Matrix4d&      Tab         //< Input:
        );


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// lookup the id of edge between start and end frame (if it exists)
    bool FindEdgeId(
            unsigned int        nStartId,       //< Input:
            unsigned int        nEndId,         //< Input:
            unsigned int&       nEdgeId         //< Output: Global ID of edge
        );


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// true if ID of frame exists
    bool FrameExists(
            unsigned int    nId     //< Input:
        );


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// true if there is an edge between start and end frame
    bool EdgeExists(
            unsigned int    nStartId,       //< Input
            unsigned int    nEndId          //< Input
        );


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// returns true if frame matching ID is a keyframe.
    bool IsKeyframe(
            unsigned int    nFrameId        //< Input
        );


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// get camera intrinsics
    Eigen::Matrix3d GetGreyCameraK(
            unsigned int    nLevel = 0      //< Input: Pyramid level intrinsic
        );


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Eigen::Matrix3d GetDepthCameraK(
            unsigned int    nLevel = 0      //< Input: Pyramid level intrinsic
        );


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Eigen::Matrix4d GetGreyCameraPose();


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Eigen::Matrix4d GetDepthCameraPose();


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// get pointer of edge given ID
    EdgePtr GetEdgePtr( unsigned int nEdgeId );


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// get pointer of edge between start and end frame
    EdgePtr GetEdgePtr( unsigned int nStartId, unsigned int nEndId );


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// get pointer of frame given ID
    FramePtr GetFramePtr( unsigned int nFrameId );


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    unsigned int GetNumFrames() { return m_vFrames.size(); }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    unsigned int GetNumEdges() { return m_vEdges.size(); }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    double GetLastModifiedTime() { return m_dLastModifiedTime; }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// set the transform between two frames
    void SetRelativeTransform( int nStartId, int nEndId, Eigen::Matrix4d& Tab );


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// get the transform between two frames
    bool GetRelativeTransform( int nStartId, int nEndId, Eigen::Matrix4d& Tab );


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// get transform between input ID and its parent
    bool GetTransformFromParent(
            unsigned int        nChildFrameId,      //< Input: Child ID we will find parent of
            Eigen::Matrix4d&    dTab                //< Output: Found transform from parent to child
        );


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// return smart pointer to frames "parent"
    FramePtr GetParentFramePtr( unsigned int nChildFrameId );


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// traverse the graph via BFS up to input depth and store absolute poses relative to input root
    void GenerateAbsolutePoses(
            std::map<unsigned int, Eigen::Matrix4d>&    vPoses,         //< Output: Poses id and absolute poses from input root
            int                                         nRootId = -1,   //< Input: Frame that will be the origin [default: last frame]
            int                                         nDepth = -1     //< Input: Grapth depth search from root [default: max depth]
        );


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// updates internal path with absolute poses relative to last frame -- also does plane fit
    void UpdateInternalPath();
    void UpdateInternalPathFull();


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// get reference to internal path variable
    std::map< unsigned int, Eigen::Matrix4d >& GetInternalPath();


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// get reference to path orientation from plane fit
    Eigen::Matrix4d& GetPathOrientation();


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// set frame as current keyframe
    void SetKeyframe(
            FramePtr    pKeyframe       //< Input: Pointer to keyframe
        );

    bool SetKeyframe(
            unsigned int    nKeyframeId       //< Input: Keyframe ID
        );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// get pointer to current keyframe
    FramePtr GetCurrentKeyframe();


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// find all the closest keyframes given an input pose and a norm -- searches through the internal path poses
    void FindClosestKeyframes(
            const Eigen::Matrix4d&                              dPose,
            float                                               fNorm,
            std::vector< std::pair< unsigned int, float > >&    vKeyframes
        );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// find all the keyframes that are similar to input thumbnail and less than score -- searches through all the map
    void FindSimilarKeyframes(
            const cv::Mat&                                      GreyThumb,      //< Input
            float                                               fMaxScore,      //< Input
            std::vector< std::pair< unsigned int, float > >&    vKeyframes      //< Output
        );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Copy changes from RHS into this
    /// TODO for large copies, this could break the front end update speed.  Fix this by making
    /// the copy process a separate thread and having it only copy for a fixed amount of time
    /// until giving up control to the front end.
    bool CopyMapChanges(
            DenseMap&       rRHS        //< Input: Map to copy from
        );


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// map functions
    void PrintMap();
    void ExportMap();
    bool ImportMap(
            const std::string&      sMap
        );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// mutex
    void Lock() { m_Mutex.lock(); }
    void Unlock() { m_Mutex.unlock(); }


private:

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// updates the map's modified time -- to be called whenever a change occurs in the map
    void _UpdateModifiedTime();


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// reset color and depth of all of the map's frames
    void _ResetNodes();


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// estimate dominant plane using PCA
    void _DynamicGroundPlaneEstimation();


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// auxilary function used for sorting elements by their distance in FindClosestKeyframes()
    static bool _CompareNorm(
            std::pair< unsigned int, float > lhs,
            std::pair< unsigned int, float > rhs
        );


    /////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
public:
    Eigen::Matrix4d                             m_dCurPose;
    Eigen::Matrix4d                             m_dPrevPose;

/////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:

    FramePtr                                    m_pCurKeyframe;
    double                                      m_dLastModifiedTime;    // crucial that we keep this up-to date

    CameraModelPyramid                          m_CModPyrGrey;
    CameraModelPyramid                          m_CModPyrDepth;

    bool                                        m_bFitPlane;
    Eigen::Matrix4d                             m_dPathOrientation;
    std::map< unsigned int, Eigen::Matrix4d >   m_vPath;                // absolute poses relative to a particular keyframe

    // actual map graph
    std::vector< FramePtr >                     m_vFrames;              // list of map's reference frames
    std::vector< EdgePtr >                      m_vEdges;               // list of map's edges

    std::mutex                                  m_Mutex;
};

#endif
