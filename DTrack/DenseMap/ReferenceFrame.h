#ifndef _REFERENCE_FRAME_H_
#define _REFERENCE_FRAME_H_

#include <vector>
#include <limits.h>

#define NO_PARENT INT_MAX

class ReferenceFrame
{
    public:

        ReferenceFrame();

        // copy constrctor
        ReferenceFrame( const ReferenceFrame& rRHS )
        {
            _Copy(rRHS);
        }

        ReferenceFrame& operator=( const ReferenceFrame& rRHS )
        {
            if( this != &rRHS ){
                _Copy(rRHS);
            }
            return *this;
        }

        void AddNeighbor( unsigned int uEdgeId );

        // Setters
        void SetId( unsigned int uId );
        void SetWhite();
        void SetGrey();
        void SetBlack();
        void SetDepth( unsigned int uDepth );
        void SetTime( double dTime );
        void SetParentEdgeId( unsigned int uEdgeId );
        void SetBrokenLink();

        // Getters
        unsigned int Id();
        unsigned int Color();
        unsigned int Depth();
        unsigned int NumNeighbors();
        unsigned int NumLandmarks();
        unsigned int NumMeasurements();
        double Time();

        unsigned int GetNeighborEdgeId( unsigned int uIdx );

        /// HACK hand out reference to our private data
        std::vector<unsigned int>& Neighbors();

        /// Return edge to parent
        unsigned int ParentEdgeId();

        // check if edge between this frame and the previous one is broken
        bool IsBroken();
        bool IsBlack();
        bool IsWhite();
        bool IsGrey();

        void _Copy( const ReferenceFrame& rRHS )
        {
            m_bBrokenLink   = rRHS.m_bBrokenLink;
            m_uColor        = rRHS.m_uColor;
            m_uDepth        = rRHS.m_uDepth;
            m_uId           = rRHS.m_uId;
            m_uParentEdgeId = rRHS.m_uParentEdgeId;
            m_dSensorTime   = rRHS.m_dSensorTime;

            // TODO only need to copy this IFF there has been a change, which could be found via the timestamp
            m_vNeighborEdgeIds.clear();
            m_vNeighborEdgeIds.insert( m_vNeighborEdgeIds.begin(),
                                       rRHS.m_vNeighborEdgeIds.begin(),
                                       rRHS.m_vNeighborEdgeIds.end() );

//            std::copy( rRHS.m_vNeighborEdgeIds.begin(), rRHS.m_vNeighborEdgeIds.end(), m_vNeighborEdgeIds.begin() );

        }


    private:

        bool                                   m_bBrokenLink;
        unsigned int                           m_uColor;
        unsigned int                           m_uDepth;
        unsigned int                           m_uId;
        unsigned int                           m_uParentEdgeId;     // for bfs
        double                                 m_dSensorTime;       // Time measurements were made
        std::vector<unsigned int>              m_vNeighborEdgeIds;  // for the co-vis graph
};

#endif

