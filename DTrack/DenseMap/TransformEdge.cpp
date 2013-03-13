#include <Mvlpp/Mvl.h>

#include "TransformEdge.h"

//////////////////////////////////////////////////////////////////////////////////////////
TransformEdge::TransformEdge(
        const unsigned int uStartId,//< Input: Frame ID of edge start
        const unsigned int uEndId,  //< Input: Frame ID of edge end
        Eigen::Matrix4d Tab  //< Input: optional transform
        )
{
    m_uStartId = uStartId;
    m_uEndId = uEndId;
    m_dTab = Tab;
}

//////////////////////////////////////////////////////////////////////////////////////////
void TransformEdge::SetTransform( Eigen::Matrix4d& Tab )
{
    m_dTab = Tab;
}

//////////////////////////////////////////////////////////////////////////////////////////
bool TransformEdge::SetTransform( unsigned int uStartId, unsigned int uEndId, Eigen::Matrix4d& Tab )
{
    if( uStartId == m_uStartId && uEndId == m_uEndId ){
        m_dTab = Tab;
        return true;
    }
    if( uEndId == m_uStartId && uStartId == m_uEndId ){
        m_dTab = mvl::TInv(Tab);
        return true;
    }
    return false;
}

//////////////////////////////////////////////////////////////////////////////////////////
bool TransformEdge::GetTransform( unsigned int uStartId, unsigned int uEndId, Eigen::Matrix4d& Tab )
{
    if( uStartId == m_uStartId && uEndId == m_uEndId ){
        Tab = m_dTab;
        return true;
    }
    if( uEndId == m_uStartId && uStartId == m_uEndId ){
        Tab = mvl::TInv(m_dTab);
        return true;
    }
    return false;
}

//////////////////////////////////////////////////////////////////////////////////////////
unsigned int TransformEdge::EndId()
{
    return m_uEndId;
}

//////////////////////////////////////////////////////////////////////////////////////////
unsigned int TransformEdge::StartId()
{
    return m_uStartId;
}

//////////////////////////////////////////////////////////////////////////////////////////
