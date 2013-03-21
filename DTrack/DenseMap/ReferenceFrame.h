#ifndef _REFERENCE_FRAME_H_
#define _REFERENCE_FRAME_H_

#include <vector>
#include <limits.h>

#include <opencv.hpp>
#include <Eigen/Dense>

#define NO_PARENT INT_MAX

class  ReferenceFrame
{
public:

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ReferenceFrame();

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ReferenceFrame(
            const ReferenceFrame&   rRHS        //< Input: Reference frame we are copying
        )
    {
        _Copy(rRHS);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ReferenceFrame& operator=(
            const ReferenceFrame&   rRHS        //< Input: Reference frame we are copying
        )
    {
        if( this != &rRHS ) {
            _Copy(rRHS);
        }
        return *this;
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void AddNeighbor(
            unsigned int    uEdgeId     //< Input: Edge that links to the new neighbor
        );


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Setters
    //

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void SetId( unsigned int uId );

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void SetTime( double dTime );

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void SetGlobalPose( const Eigen::Matrix4d& dPose );

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void SetImages(
            const cv::Mat&      GreyImage,      //< Input: Greyscale image
            const cv::Mat&      DepthImage      //< Input: Depth image
        );

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void SetThumbs(
            const cv::Mat&      GreyThumb,      //< Input: Greyscale image
            const cv::Mat&      DepthThumb      //< Input: Depth image
        );

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void SetParentEdgeId( unsigned int uEdgeId );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Getters
    //

    unsigned int GetId();
    double GetTime();
    Eigen::Matrix4d GetGlobalPose();
    unsigned int GetNumNeighbors();

    unsigned int GetNeighborEdgeId( unsigned int uIdx );

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void GetImages(
            cv::Mat&      GreyImage,      //< Output: Greyscale image
            cv::Mat&      DepthImage      //< Output: Depth image
        );

    int GetImageHeight() { return m_GreyImage.rows; }
    int GetThumbHeight() { return m_GreyThumb.rows; }
    int GetImageWidth() { return m_GreyImage.cols; }
    int GetThumbWidth() { return m_GreyThumb.cols; }

    unsigned char* GetGreyImagePtr() { return m_GreyImage.data; }
    unsigned char* GetGreyThumbPtr() { return m_GreyThumb.data; }
    unsigned char* GetDepthImagePtr() { return m_DepthImage.data; }
    unsigned char* GetDepthThumbPtr() { return m_DepthThumb.data; }

    /// HACK hand out reference to our private data
    std::vector<unsigned int>& GetNeighbors();

    // return edge to parent
    unsigned int GetParentEdgeId();


private:

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void _Copy(
            const ReferenceFrame&       rRHS        //< Input: Reference frame we are copying
        );


/////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
    double                                  m_dSensorTime;          // time measurements were made
    unsigned int                            m_nId;                  // reference frame ID
    unsigned int                            m_nParentEdgeId;        // for bfs
    std::vector< unsigned int >             m_vNeighborEdgeIds;     // for the co-vis graph

    // TODO this has to be replaced with a method that computes a "global" pose given a starting
    // frame and a DFS
    Eigen::Matrix4d                         m_dGlobalPose;          // keyframe's global pose

    cv::Mat                                 m_GreyImage;
    cv::Mat                                 m_GreyThumb;

    cv::Mat                                 m_DepthImage;
    cv::Mat                                 m_DepthThumb;

};

#endif
