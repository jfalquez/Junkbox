#include <math.h>

#include <tuple>
#include <algorithm>

#include "Thumbnails.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Thumbnails::Thumbnails() :
    m_nImgSize(0), m_fGreyThreshold(5.0), m_fDepthThreshold(10.0), m_fGreyDepthRatio(1.0)
{
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template < typename T >
void GenerateThumbnail(
        T*                  pImg,
        unsigned int&       nWidth,
        unsigned int&       nHeight,
        unsigned int        nFactor
    )
{
    unsigned int nThumbWidth = nWidth << nFactor;
    unsigned int nThumbHeight = nHeight << nFactor;

    T* pBuff = new T[nThumbHeight*nThumbWidth];


}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Thumbnails::SetGreyThreshold( float fThreshold )
{
    m_fGreyThreshold = fThreshold;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Thumbnails::SetDepthThreshold( float fThreshold )
{
    m_fDepthThreshold = fThreshold;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Thumbnails::SetGreyDepthRatio( float fRatio )
{
    if( fRatio > 0 && fRatio < 1.0 ) {
        m_fGreyDepthRatio = fRatio;
        return true;
    }
    return false;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Thumbnails::PushThumbnails(
        unsigned int        nId,
        unsigned int        nSize,
        unsigned char*      pGrey,
        float*              pDepth
    )
{
    // grey thumbnails are mandatory
    if( !pGrey ) {
        return false;
    }

    // check if a thumbnail with this ID already exists
    if( m_ThumbsDepth.find( nId ) != m_ThumbsDepth.end() ) {
        return false;
    }

    // store image size
    if( m_nImgSize == 0 ) {
        m_nImgSize = nSize;
    }

    // verify image size
    if( nSize != m_nImgSize ) {
        return false;
    }

    m_ThumbsGrey[nId] = pGrey;


    // depth thumbnails are optional
    if( pDepth ) {

    }

    return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Thumbnails::FindBestMatch(
        unsigned char*                  pGrey,
        float*                          pDepth,
        std::vector< unsigned int >&    vOut
    )
{
    if( m_ThumbsGrey.empty() ) {
        return false;
    }

    std::vector< std::pair< unsigned int, float > >    vMatches;

    float fThresholdGrey = m_fGreyThreshold * m_nImgSize;
    for( auto ii = m_ThumbsGrey.begin(); ii != m_ThumbsGrey.end(); ++ii ) {
        const unsigned int& nIdx = ii->first;

        float fGreyScore = _ScoreImages<unsigned char>( pGrey, m_ThumbsGrey[nIdx], m_nImgSize);
        if( fGreyScore < fThresholdGrey ) {
            vMatches.push_back( std::pair<unsigned int, float>( nIdx, fGreyScore ) );
            printf("=== SCORE: %f\n", fGreyScore );
        }

    }

    // sort by most similar
    std::sort( vMatches.begin(), vMatches.end(), _CompareNorm );

    // store vector
    for( int ii = 0; ii < vMatches.size(); ++ii ) {
        vOut.push_back( std::get<0>( vMatches[ii] ) );
    }

    return true;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


template < typename T >
inline float Thumbnails::_ScoreImages(
        T*              pImg1,
        T*              pImg2,
        unsigned int    nSize
        )
{
    float fScore = 0;
    for( unsigned int ii = 0; ii < nSize; ++ii ) {
        fScore += fabs( pImg1[ii] - pImg2[ii] );
    }
    return fScore;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Thumbnails::_CompareNorm(
        std::pair< unsigned int, float > lhs,
        std::pair< unsigned int, float > rhs
    )
{
    return std::get<1>(lhs) < std::get<1>(rhs);
}


