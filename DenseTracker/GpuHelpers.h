#ifndef GPU_HELPERS_H
#define GPU_HELPERS_H

#include <kangaroo/kangaroo.h>


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct GpuVars_t
{
    GpuVars_t( unsigned int nImgHeight, unsigned int nImgWidth ) :
        LeftPyr( nImgWidth, nImgHeight ),
        RightPyr( nImgWidth, nImgHeight ),
        KeyPyr( nImgWidth, nImgHeight ),
        KeyDepthPyr( nImgWidth, nImgHeight ),
        KeyDepthPyrNormalized( nImgWidth, nImgHeight ),
        Workspace( nImgWidth * sizeof(Gpu::LeastSquaresSystem<float,6>), nImgHeight ),
        Debug( nImgWidth, nImgHeight )
        ,uTmpPyr1( nImgWidth, nImgHeight ),
        fTmpPyr1( nImgWidth, nImgHeight ),
        fTmpPyr2( nImgWidth, nImgHeight ),
        uTmp1( nImgWidth, nImgHeight ),
        uTmp2( nImgWidth, nImgHeight )
    { }

    // member variables
    Gpu::Pyramid< unsigned char, MAX_PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage >   LeftPyr;
    Gpu::Pyramid< unsigned char, MAX_PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage >   RightPyr;
    Gpu::Pyramid< unsigned char, MAX_PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage >   KeyPyr;
    Gpu::Pyramid< float, MAX_PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage >           KeyDepthPyr;
    Gpu::Pyramid< float, MAX_PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage >           KeyDepthPyrNormalized;
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage >                     Workspace;
    Gpu::Image<float4, Gpu::TargetDevice, Gpu::Manage>                              Debug;

    // temporal auxilary variables (by type)
    Gpu::Pyramid< unsigned char, MAX_PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage >   uTmpPyr1;
    Gpu::Pyramid< float, MAX_PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage >           fTmpPyr1;
    Gpu::Pyramid< float, MAX_PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage >           fTmpPyr2;
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage >                     uTmp1;
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage >                     uTmp2;

};


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


/* */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GenerateThumbnail(
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
/* */

#endif // GPU_HELPERS_H
