#pragma once

#include <map>
#include <vector>

class Thumbnails
{
public:

    // TODO set length of "history" to look over
    Thumbnails();

    // TODO add search radius for poses

    // TODO add method that generates thumbnails for user
    template < typename T >
    void GenerateThumbnail(
            T*                  pImg,       //< Input/Output
            unsigned int&       nWidth,     //< Input/Output
            unsigned int&       nHeight,    //< Input/Output
            unsigned int        nFactor     //< Input
        );

    void SetGreyThreshold( float fThreshold );

    void SetDepthThreshold( float fThreshold );

    // TODO make this better since a plain ratio cannot be
    // used given that depth vs intensities are not comparable directly
    bool SetGreyDepthRatio( float fRatio );

    bool PushThumbnails(
            unsigned int        nId,                //< Input
            unsigned int        nSize,              //< Input
            unsigned char*      pGrey,              //< Input
            float*              pDepth = nullptr    //< Input
        );

    bool FindBestMatch(
            unsigned char*                  pGrey,      //< Input
            float*                          pDepth,     //< Input
            std::vector< unsigned int >&    vOut        //< Output
        );


private:

    template < typename T >
    inline float _ScoreImages(
            T*              pImg1,
            T*              pImg2,
            unsigned int    nSize
        );

    static bool _CompareNorm(
            std::pair< unsigned int, float > lhs,
            std::pair< unsigned int, float > rhs
        );


private:

    unsigned int                                    m_nImgSize;

    float                                           m_fGreyThreshold;
    float                                           m_fDepthThreshold;
    float                                           m_fGreyDepthRatio;


    // TODO use deque?
    // TODO pack everything into single structure
    std::map< unsigned int, unsigned char* >        m_ThumbsGrey;
    std::map< unsigned int, float* >                m_ThumbsDepth;
    std::map< unsigned int, float* >                m_ThumbsPose;


};
