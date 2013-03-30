#ifndef GPU_HELPERS_H
#define GPU_HELPERS_H

#include <kangaroo/kangaroo.h>

#include "DenseFrontEndConfig.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct GpuVars_t
{
    GpuVars_t( unsigned int nImgWidth, unsigned int nImgHeight ) :
        uPyr1( nImgWidth, nImgHeight ),
        fPyr1( nImgWidth, nImgHeight ),
        fPyr2( nImgWidth, nImgHeight ),
        uImg1( nImgWidth, nImgHeight ),
        uImg2( nImgWidth, nImgHeight )
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




/*

namespace gphp {

Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage >     WORKSPACE;
const unsigned int                                              MAX_PYRAMID_LEVEL = MAX_PYR_LEVELS;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void Init( unsigned int nSize )
{
    WORKSPACE = Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage >( nSize, 1 );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void GenerateThumbnail(
        const cv::Mat&                  Image,              //< Input: Original image
        cv::Mat&                        ThumbImage,         //< Output: Thumbnail image
        Gpu::Image<unsigned char>       Wksp = WORKSPACE    //< Input: GPU Memory Workspace
        )
{
    const unsigned int nImageWidth  = Image.cols;
    const unsigned int nImageHeight = Image.rows;
    const unsigned int nThumbWidth  = ThumbImage.cols;

    if( Wksp.Area() == 0 ) {
        std::cerr << "warning: Workspace has not been initialized!" << std::endl;
        return;
    }

    if( nImageWidth % nThumbWidth != 0 ) {
        std::cerr << "warning: Image is not divisible by Thumbnail size!" << std::endl;
        return;
    }

    const unsigned int nPyrLvl = (log( nImageWidth / nThumbWidth ) / 0.301) - 1;
    assert( nPyrLvl <= MAX_PYRAMID_LEVEL );

    Gpu::Pyramid< unsigned char, MAX_PYRAMID_LEVEL, Gpu::TargetDevice, Gpu::DontManage > Pyr;
    Pyr.AllocateFromImage( nImageWidth, nImageHeight, Wksp );

    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::DontManage > Img1;
    Img1 = Wksp.SplitAlignedImage<unsigned char>( nImageWidth, nImageHeight );

    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::DontManage > Img2;
    Img2 = Wksp.SplitAlignedImage<unsigned char>( nImageWidth, nImageHeight );

    // load image to GPU
    Pyr[0].MemcpyFromHost( Image.data, nImageWidth );
    Gpu::BlurReduce< unsigned char, MAX_PYRAMID_LEVEL, unsigned int >( Pyr, Img1, Img2 );
    Pyr[nPyrLvl-1].MemcpyToHost( ThumbImage.data, nThumbWidth );
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void GenerateDepthThumbnail(
        const cv::Mat&                  Image,              //< Input: Original image
        cv::Mat&                        ThumbImage,         //< Output: Thumbnail image
        Gpu::Image<unsigned char>       Wksp = WORKSPACE    //< Input: GPU Memory Workspace
        )
{
    const unsigned int nImageWidth  = Image.cols;
    const unsigned int nImageHeight = Image.rows;
    const unsigned int nThumbWidth  = ThumbImage.cols;

    if( Wksp.Area() == 0 ) {
        std::cerr << "warning: Workspace has not been initialized!" << std::endl;
        return;
    }

    if( nImageWidth % nThumbWidth != 0 ) {
        std::cerr << "warning: Image is not divisible by Thumbnail size!" << std::endl;
        return;
    }

    const unsigned int nPyrLvl = (log( nImageWidth / nThumbWidth ) / 0.301) - 1;
    assert( nPyrLvl <= MAX_PYRAMID_LEVEL );

    Gpu::Pyramid< unsigned char, MAX_PYRAMID_LEVEL, Gpu::TargetDevice, Gpu::DontManage > Pyr;
    Pyr.AllocateFromImage( nImageWidth, nImageHeight, Wksp );

    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::DontManage > Img1;
    Img1 = Wksp.SplitAlignedImage<unsigned char>( nImageWidth, nImageHeight );

    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::DontManage > Img2;
    Img2 = Wksp.SplitAlignedImage<unsigned char>( nImageWidth, nImageHeight );


    // load image to GPU
    Pyr[0].MemcpyFromHost( Image.data, nImageWidth );
    Gpu::BlurReduce< unsigned char, MAX_PYRAMID_LEVEL, unsigned int >( Pyr, Img1, Img2 );
    Pyr[nPyrLvl-1].MemcpyToHost( ThumbImage.data, nThumbWidth );
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Disp2Depth(
        cv::Mat&                        Image,              //< Input/Output: Disparity image, Depth image.
        float                           fu,                 //< Input: Focal length
        float                           baseline,           //< Input: Baseline
        Gpu::Image<unsigned char>       Wksp = WORKSPACE    //< Input: GPU Memory Workspace
        )
{
    const unsigned int nImageWidth  = Image.cols;
    const unsigned int nImageHeight = Image.rows;

    Gpu::Image< float, Gpu::TargetDevice, Gpu::DontManage > dImage;
    dImage = Wksp.SplitAlignedImage<float>( nImageWidth, nImageHeight );

    dImage.MemcpyFromHost( Image.data );

    Gpu::Disp2Depth( dImage, dImage, fu, baseline );

    dImage.MemcpyToHost( Image.data );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

} /* namespace */


/*
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void GenerateThumbnail(
        GpuVars_t&                      dVars,              //< Input: GPU Workspace
        const cv::Mat&                  Image,              //< Input: Original image
        cv::Mat&                        ThumbImage          //< Output: Thumbnail image
        )
{
    // load image to GPU
    dVars.uTmpPyr1[0].MemcpyFromHost( Image.data );
    Gpu::BlurReduce< unsigned char, MAX_PYR_LEVELS, unsigned int >( dVars.uTmpPyr1, dVars.uTmp1, dVars.uTmp2 );
    dVars.uTmpPyr1[MAX_PYR_LEVELS-1].MemcpyToHost( ThumbImage.data );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void GenerateDepthThumbnail(
        GpuVars_t&                      dVars,              //< Input: GPU Workspace
        const cv::Mat&                  Image,              //< Input: Original image
        cv::Mat&                        ThumbImage          //< Output: Thumbnail image
        )
{
    // upload depth image
    dVars.fTmpPyr1[0].MemcpyFromHost( Image.data );

    // downsample depth map
    Gpu::BoxReduce< float, MAX_PYR_LEVELS, float >( dVars.fTmpPyr1 );


    // cross-bilateral filter the downsampled depth maps
    if( g_bBiFilterThumbs == true ) {
        dVars.fTmpPyr2[0].CopyFrom( dVars.fTmpPyr1[0] );
        for(int ii = 1; ii < MAX_PYR_LEVELS; ii++ ) {
            Gpu::BilateralFilter< float, float, unsigned char >( dVars.fTmpPyr2[ii], dVars.fTmpPyr1[ii], dVars.uTmpPyr1[ii],
                                                               g_dThumbFiltS, g_dThumbFiltD, g_dThumbFiltC, g_nThumbFiltSize );
        }
        dVars.fTmpPyr1.CopyFrom( dVars.fTmpPyr2 );
    }

    dVars.fTmpPyr1[MAX_PYR_LEVELS-1].MemcpyToHost( ThumbImage.data );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Disp2Depth(
        GpuVars_t&                      dVars,              //< Input: GPU Workspace
        cv::Mat&                        Image,              //< Input/Output: Disparity image, Depth image.
        float                           fu,                 //< Input: Focal length
        float                           baseline            //< Input: Baseline
        )
{
    Gpu::Image< float, Gpu::TargetDevice, Gpu::Manage >& dImage = dVars.fTmpPyr1[0];

    dImage.MemcpyFromHost( Image.data );

    Gpu::Disp2Depth( dImage, dImage, fu, baseline );

    dImage.MemcpyToHost( Image.data );
}


/* */

#endif // GPU_HELPERS_H
