#ifndef GPU_HELPERS_H
#define GPU_HELPERS_H


#pragma GCC diagnostic ignored "-Wreorder"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <kangaroo/kangaroo.h>
#pragma GCC diagnostic pop

#include "DenseFrontEndConfig.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct GpuVars_t
{
    GpuVars_t( unsigned int nImgWidth, unsigned int nImgHeight ) :
        uImg1( nImgWidth, nImgHeight ),
        uImg2( nImgWidth, nImgHeight ),
        uPyr1( nImgWidth, nImgHeight ),
        fPyr1( nImgWidth, nImgHeight ),
        fPyr2( nImgWidth, nImgHeight )
    { }

    // temporal auxilary variables (by type)
    Gpu::Image<unsigned char, Gpu::TargetDevice, Gpu::Manage>                       uImg1;
    Gpu::Image<unsigned char, Gpu::TargetDevice, Gpu::Manage>                       uImg2;
    Gpu::Pyramid<unsigned char, MAX_PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage>     uPyr1;
    Gpu::Pyramid<float, MAX_PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage>             fPyr1;
    Gpu::Pyramid<float, MAX_PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage>             fPyr2;
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// performs CUDA memory check and returns amount of free GPU memory
unsigned int CheckMemoryCUDA();


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// converts disparities into depth values
inline void Disp2Depth(
        GpuVars_t&                      Scrap,              //< Input: GPU workspace
        float                           fFocalLength,       //< Input: Focal length
        float                           fBaseline,          //< Input: Baseline
        cv::Mat&                        Image               //< Input/Output: Disparity image, Depth image.
        )
{
    Gpu::Image< float, Gpu::TargetDevice, Gpu::Manage >& dImage = Scrap.fPyr1[0];

    dImage.MemcpyFromHost( Image.data );

    Gpu::Disp2Depth( dImage, dImage, fFocalLength, fBaseline );

    dImage.MemcpyToHost( Image.data );
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// generates a thumbnail given an input image - the thumbnail is the smallest resolution in the pyramid
inline void GenerateGreyThumbnail(
        GpuVars_t&                      Scrap,              //< Input: GPU workspace
        const cv::Mat&                  Image,              //< Input: Original image
        cv::Mat&                        ThumbImage          //< Output: Thumbnail image
        )
{
    // upload grey image
    Scrap.uPyr1[0].MemcpyFromHost( Image.data );

    // reduce
    Gpu::BlurReduce<unsigned char, MAX_PYR_LEVELS, unsigned int>( Scrap.uPyr1, Scrap.uImg1, Scrap.uImg2 );

    // copy image from GPU
    assert( ThumbImage.empty() == false );
    Scrap.uPyr1[MAX_PYR_LEVELS-1].MemcpyToHost( ThumbImage.data );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// generates a thumbnail given an input image - the thumbnail is the smallest resolution in the pyramid
inline void GenerateDepthThumbnail(
        GpuVars_t&                      Scrap,              //< Input: GPU workspace
        const cv::Mat&                  Image,              //< Input: Original image
        cv::Mat&                        ThumbImage          //< Output: Thumbnail image
        )
{
    // upload depth image
    Scrap.fPyr1[0].MemcpyFromHost( Image.data );

    // downsample depth map
    Gpu::BoxReduce< float, MAX_PYR_LEVELS, float >( Scrap.fPyr1 );

    /*
    TODO enable this if it is worth it
    // (optional) cross-bilateral filter the downsampled depth maps
    if( g_bBiFilterThumbs == true ) {
        Wksp.fPyr2[0].CopyFrom( Wksp.fPyr1[0] );
        for(int ii = 1; ii < MAX_PYR_LEVELS; ++ii ) {
            Gpu::BilateralFilter< float, float, unsigned char >( Wksp.fPyr2[ii], Wksp.fPyr1[ii], Wksp.uPyr1[ii],
                                                               g_dThumbFiltS, g_dThumbFiltD, g_dThumbFiltC, g_nThumbFiltSize );
        }
        Wksp.fPyr1.CopyFrom( Wksp.fPyr2 );
    }
    */

    assert( ThumbImage.empty() == false );
    Scrap.fPyr1[MAX_PYR_LEVELS-1].MemcpyToHost( ThumbImage.data );
}


#endif // GPU_HELPERS_H
