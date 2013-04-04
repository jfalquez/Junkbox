#ifndef _TRANSFORM_EDGE_H_
#define _TRANSFORM_EDGE_H_

#include <Eigen/Dense>

class TransformEdge
{
public:

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    TransformEdge(
            const unsigned int          nStartId,                           //< Input: Frame ID of edge start
            const unsigned int          nEndId,                             //< Input: Frame ID of edge end
            Eigen::Matrix4d             Tse = Eigen::Matrix4d::Identity()   //< Input: Optional transform
            );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    TransformEdge(
            const TransformEdge&    rRHS        //< Input: Transform edge we are copying
        )
    {
        m_nStartId  = rRHS.m_nStartId;
        m_nEndId    = rRHS.m_nEndId;
        m_dTse      = rRHS.m_dTse;
        m_dTse_orig = rRHS.m_dTse_orig;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    TransformEdge& operator=(
            const TransformEdge&    rRHS        //< Input: Transform edge we are copying
        )
    {
        if( &rRHS == this ){
            return *this;
        }
        m_nStartId  = rRHS.m_nStartId;
        m_nEndId    = rRHS.m_nEndId;
        m_dTse      = rRHS.m_dTse;
        m_dTse_orig = rRHS.m_dTse_orig;
        return *this;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void SetTransform(
            const Eigen::Matrix4d&  Tse         //< Input: Transform
        );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool SetTransform(
            unsigned int            nStartId,       //< Input: Frame ID of edge start
            unsigned int            nEndId,         //< Input: Frame ID of edge end
            const Eigen::Matrix4d&  Tab             //< Input: Transform
        );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool GetTransform(
            unsigned int            nStartId,       //< Input: Frame ID of edge start
            unsigned int            nEndId,         //< Input Frame ID of edge end
            Eigen::Matrix4d&        Tse             //< Output: Transform
        );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    unsigned int GetEndId();

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    unsigned int GetStartId();

/////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
    unsigned int            m_nStartId;
    unsigned int            m_nEndId;
    Eigen::Matrix4d         m_dTse;             // transform
    Eigen::Matrix4d         m_dTse_orig;        // original transform estimated
};

#endif
