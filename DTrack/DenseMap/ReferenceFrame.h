#ifndef _REFERENCE_FRAME_H_
#define _REFERENCE_FRAME_H_

#include <vector>
#include <limits.h>

#include <opencv.hpp>

#define NO_PARENT INT_MAX

class ReferenceFrame
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
    void SetImages(
            const cv::Mat&      GreyImage,      //< Input: Greyscale image
            const cv::Mat&      DepthImage      //< Input: Depth image
        );

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void SetParentEdgeId( unsigned int uEdgeId );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Getters
    //

    unsigned int Id();
    double Time();
    unsigned int NumNeighbors();

    unsigned int GetNeighborEdgeId( unsigned int uIdx );

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void GetImages(
            cv::Mat&      GreyImage,      //< Output: Greyscale image
            cv::Mat&      DepthImage      //< Output: Depth image
        );

    /// HACK hand out reference to our private data
    std::vector<unsigned int>& Neighbors();

    // return edge to parent
    unsigned int ParentEdgeId();


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

    cv::Mat                                 m_GreyImage;
    cv::Mat                                 m_GreyThumb;

    cv::Mat                                 m_DepthImage;
    cv::Mat                                 m_DepthThumb;

};

#endif
