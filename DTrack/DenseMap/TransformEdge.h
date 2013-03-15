#ifndef _TRANSFORM_EDGE_H_
#define _TRANSFORM_EDGE_H_

#include <Eigen/Dense>

class TransformEdge
{
    public:

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        TransformEdge(
                const unsigned int          uStartId,                           //< Input: Frame ID of edge start
                const unsigned int          uEndId,                             //< Input: Frame ID of edge end
                Eigen::Matrix4d             Tab = Eigen::Matrix4d::Identity()   //< Input: Optional transform
                );

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        TransformEdge(
                const TransformEdge&    rRHS        //< Input: Transform edge we are copying
            )
        {
            m_uStartId  = rRHS.m_uStartId;
            m_uEndId    = rRHS.m_uEndId;
            m_dTab      = rRHS.m_dTab;
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        TransformEdge& operator=(
                const TransformEdge&    rRHS        //< Input: Transform edge we are copying
            )
        {
            if( &rRHS == this ){
                return *this;
            }
            m_uStartId  = rRHS.m_uStartId;
            m_uEndId    = rRHS.m_uEndId;
            m_dTab      = rRHS.m_dTab;
            return *this;
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void SetTransform(
                const Eigen::Matrix4d&  Tab         //< Input: Transform
            );

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool SetTransform(
                unsigned int            uStartId,       //< Input: Frame ID of edge start
                unsigned int            uEndId,         //< Input: Frame ID of edge end
                const Eigen::Matrix4d&  Tab             //< Input: Transform
            );

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool GetTransform(
                unsigned int            uStartId,       //< Input: Frame ID of edge start
                unsigned int            uEndId,         //< Input Frame ID of edge end
                Eigen::Matrix4d&        Tab             //< Output: Transform
            );

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        unsigned int EndId();

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        unsigned int StartId();


    private:

        unsigned int           m_uStartId;
        unsigned int           m_uEndId;
        Eigen::Matrix4d        m_dTab;
};

#endif
