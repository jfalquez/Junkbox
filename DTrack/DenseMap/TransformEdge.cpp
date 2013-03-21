#include <Mvlpp/Mvl.h>

#include "TransformEdge.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TransformEdge::TransformEdge(
        const unsigned int      nStartId,   //< Input: Frame ID of edge start
        const unsigned int      nEndId,     //< Input: Frame ID of edge end
        Eigen::Matrix4d         Tab         //< Input: Optional transform
    )
{
    m_nStartId  = nStartId;
    m_nEndId    = nEndId;
    m_dTab      = Tab;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void TransformEdge::SetTransform(
        const Eigen::Matrix4d&      Tab      //< Input:
    )
{
    m_dTab = Tab;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool TransformEdge::SetTransform(
        unsigned int                nStartId,   //< Input:
        unsigned int                nEndId,     //< Input:
        const Eigen::Matrix4d&      Tab         //< Input:
    )
{
    if( nStartId == m_nStartId && nEndId == m_nEndId ){
        m_dTab = Tab;
        return true;
    }
    if( nEndId == m_nStartId && nStartId == m_nEndId ){
        m_dTab = mvl::TInv(Tab);
        return true;
    }
    return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool TransformEdge::GetTransform(
        unsigned int            nStartId,   //< Input:
        unsigned int            nEndId,     //< Input:
        Eigen::Matrix4d&        Tab         //< Output:
    )
{
    if( nStartId == m_nStartId && nEndId == m_nEndId ){
        Tab = m_dTab;
        return true;
    }
    if( nEndId == m_nStartId && nStartId == m_nEndId ){
        Tab = mvl::TInv(m_dTab);
        return true;
    }
    return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int TransformEdge::GetEndId()
{
    return m_nEndId;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int TransformEdge::GetStartId()
{
    return m_nStartId;
}
