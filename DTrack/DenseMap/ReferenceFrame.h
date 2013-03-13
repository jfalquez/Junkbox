
/*
   \file ReferenceFrame.h

   This class represents a coordinate frame on the manifold, usually coincident with 
   the vehicle or primary sensor.

 */

#ifndef _REFERENCE_FRAME_H_
#define _REFERENCE_FRAME_H_

#include <Utils/MathTypes.h>
#include <Map/Measurement.h>
#include <Map/Landmark.h>
#include <vector>

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

        /// Return a reference to the vector of landmarks for this frame
        std::vector<Landmark>& GetLandmarksVectorRef( );

        Eigen::Vector4d& LandmarkPos( unsigned int uLandmarkIndex );

        Eigen::Vector4d& LandmarkWorkRef( unsigned int uLandmarkIndex );

        Landmark&    GetLandmark( unsigned int uLandmarkIndex );

        /// Return edge to parent
        unsigned int ParentEdgeId();

        // check if edge between this frame and the previous one is broken
        bool IsBroken();
        bool IsBlack();
        bool IsWhite();
        bool IsGrey();

        /// Return a reference to the vector of measurements in a particular camera
        std::vector<Measurement>& GetMeasurementsVectorRef();

        std::vector<Measurement> GetMeasurementsVector();
        
        Measurement& GetMeasurement( unsigned int uMeasurementIndex );

        Measurement& GetMeasurement( MeasurementId& zId );

    
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
            
            m_vLandmarks.clear();
            m_vLandmarks.insert( m_vLandmarks.begin(), 
                                 rRHS.m_vLandmarks.begin(),
                                 rRHS.m_vLandmarks.end() );
//            std::copy( rRHS.m_vLandmarks.begin(), rRHS.m_vLandmarks.end(), m_vLandmarks.begin() );
            
            m_vMeasurements.clear();
            m_vMeasurements.insert( m_vMeasurements.begin(), 
                                    rRHS.m_vMeasurements.begin(),
                                    rRHS.m_vMeasurements.end() );
//            std::copy( rRHS.m_vMeasurements.begin(), rRHS.m_vMeasurements.end(), m_vMeasurements.begin() );
        }

    
    private:

        bool                                   m_bBrokenLink;
        unsigned int                           m_uColor;
        unsigned int                           m_uDepth;
        unsigned int                           m_uId;
        unsigned int                           m_uParentEdgeId;     // for bfs
        double                                 m_dSensorTime;       // Time measurements were made
        std::vector<unsigned int>              m_vNeighborEdgeIds;  // for the co-vis graph
        std::vector<Landmark>                  m_vLandmarks;        // vector of initialized landmarks at this frame
        std::vector<Measurement>               m_vMeasurements;     // vector of landmarks' image measurements 
};

#endif

