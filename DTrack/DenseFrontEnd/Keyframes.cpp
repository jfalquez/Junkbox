#ifndef KEYFRAMES_H
#define KEYFRAMES_H

#include <kangaroo/kangaroo.h>
#include <kangaroo/../applications/common/CameraModelPyramid.h>
/*
#include "Common.h"
#include "GpuHelpers.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Keyframe
struct Keyframe_t {
    Eigen::Matrix4d     Pose;
    cv::Mat             Image;
    cv::Mat             Depth;
    cv::Mat             ThumbImage;
    cv::Mat             ThumbDepth;
    float               ThumbScore;
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Score Image with Keyframe Image
inline float ScoreImages(
        const cv::Mat&              Image1,
        const cv::Mat&              Image2
        )
{
    float fScore = 0;
    for( int ii = 0; ii < Image1.rows; ii++ ) {
        for( int jj = 0; jj < Image1.cols; jj++ ) {
            fScore += abs(Image1.at<unsigned char>(ii,jj) - Image2.at<unsigned char>(ii,jj));
        }
    }
    return fScore;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Render Keyframes
inline void RenderKeyframes(
        const std::vector< Keyframe_t >&            vKeyframes,
        float                                       fThreshold
        )
{
    glPushAttrib( GL_ENABLE_BIT );
    glDisable( GL_LIGHTING );
    glEnable( GL_DEPTH_TEST );
    glEnable( GL_POINT_SMOOTH );
    glEnable( GL_BLEND );
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
    glPointSize( 5.0 );
    glBegin( GL_POINTS );
    for( int ii = 0; ii < (int)vKeyframes.size(); ii++ ) {
        if( vKeyframes[ii].ThumbScore < fThreshold ) {
            glColor4f( 1.0, 1.0, 1.0, 1.0 );
        } else {
            glColor4f( 1.0, 0.0, 0.0, 0.6 );
        }
        glVertex3f( vKeyframes[ii].Pose(0,3), vKeyframes[ii].Pose(1,3), vKeyframes[ii].Pose(2,3) );
    }
    glEnd();
    glPopAttrib();

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Find closest keyframe based on pose
inline unsigned int FindClosestKeyframe(
        const std::vector< Keyframe_t >& vKeyframes,        //< Input: Vector of keyframes
        Eigen::Matrix4d Pose                                //< Input: Pose in VISION frame
        )
{
    Eigen::Matrix4d T_wr = Pose * g_Tvr;

    unsigned int Idx = 0;

    Eigen::Vector6d PoseError = mvl::T2Cart(mvl::TInv( T_wr ) * vKeyframes[0].Pose);
    float fScore = PoseError.norm();
    float fBestScore = fScore;
    for( int ii = 1; ii < vKeyframes.size(); ii++ ) {
        PoseError = mvl::T2Cart(mvl::TInv(T_wr) * vKeyframes[ii].Pose);
        fScore = PoseError.norm();
        if( fScore < fBestScore ) {
            fBestScore = fScore;
            Idx = ii;
        }
    }
//    std::cout << "Closest Keyframe: " << Idx << std::endl;
    return Idx;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Search through nearby (potentially all) keyframes for best matching against given thumbnail
inline unsigned int FindBestKeyframe(
        std::vector< Keyframe_t >&          vKeyframes,         //< Input: Vector of keyframes
        const cv::Mat&                      ThumbImage,         //< Input: Decimated greyscale image
        const cv::Mat&                      ThumbDepth          //< Input: Decimated depth image
        )
{
    // get GUI variables
    pangolin::Var<float>            ui_fRelocalizationRatio("ui.Reloc Luminance:Depth Ratio");

    float fLumContrib   = ui_fRelocalizationRatio;
    float fDepthContrib = 1.0 - fLumContrib;

    unsigned int nBestIdx = 0;

    float ImageScore = ScoreImages( ThumbImage, vKeyframes[0].ThumbImage );
    float DepthScore = ScoreImages( ThumbDepth, vKeyframes[0].ThumbDepth );
    vKeyframes[0].ThumbScore = ( fLumContrib * ImageScore ) + ( fDepthContrib * DepthScore );
    float fBestScore     = vKeyframes[0].ThumbScore;
    for( int ii = 1; ii < vKeyframes.size(); ii++ ) {
        ImageScore = ScoreImages( ThumbImage, vKeyframes[ii].ThumbImage );
        DepthScore = ScoreImages( ThumbDepth, vKeyframes[ii].ThumbDepth );
        vKeyframes[ii].ThumbScore = ( fLumContrib * ImageScore ) + ( fDepthContrib * DepthScore );
        if( vKeyframes[ii].ThumbScore < fBestScore ) {
            fBestScore = vKeyframes[ii].ThumbScore;
            nBestIdx = ii;
        }
    }

//    std::cout << "Best Keyframe: " << nBestIdx << std::endl;
    return nBestIdx;
}

/*
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Create a new keyframe
Keyframe_t CreateKeyframe(
        GpuVars_t&                              dVars,              //< Input: GPU Workspace
        const cv::Mat&                          Image,              //< Input: Grey Image
        const cv::Mat&                          Depth,              //< Input: Depth Image
        const Eigen::Matrix4d&                  Pose                //< Input: Global Pose of this Keyframe
        )
{
    const unsigned int nImgHeight   = Image.rows;
    const unsigned int nImgWidth    = Image.cols;
    const unsigned int nThumbHeight = nImgHeight >> MAX_PYR_LEVELS-1;
    const unsigned int nThumbWidth  = nImgWidth >> MAX_PYR_LEVELS-1;

    Keyframe_t KF;

    //------------------------------------------- Pose
    KF.Pose = Pose;


    //------------------------------------------- RGB Data

    KF.Image        = Image;
    KF.ThumbImage   = cv::Mat( nThumbHeight, nThumbWidth, CV_8UC1 );

    GenerateThumbnail( dVars, Image, KF.ThumbImage );


    //------------------------------------------- Depth Data

    KF.Depth        = Depth;
    KF.ThumbDepth   = cv::Mat( nThumbHeight, nThumbWidth, CV_32FC1 );

    GenerateDepthThumbnail( dVars, Depth, KF.ThumbDepth );



    return KF;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Create a new keyframe
Keyframe_t CreateKeyframe(
        GpuVars_t&                                      dVars,      //< Input: GPU Workspace
        const CameraModelPyramid&                       CamModel,   //< Input: Camera Model Pyramid
        const std::vector< rpg::ImageWrapper >&         vImgs,      //< Input: Images
        const Eigen::Matrix4d&                          Pose        //< Input: Global Pose of this Keyframe
        )
{
    const unsigned int nImgHeight   = CamModel.Height();
    const unsigned int nImgWidth    = CamModel.Width();
    const unsigned int nThumbHeight = nImgHeight >> MAX_PYR_LEVELS-1;
    const unsigned int nThumbWidth  = nImgWidth >> MAX_PYR_LEVELS-1;

    Keyframe_t KF;

    //------------------------------------------- Pose
    KF.Pose = Pose;


    //------------------------------------------- RGB Data

    dVars.uTmpPyr1[0].MemcpyFromHost( vImgs[0].Image.data );

    // downsample greyscale image
    Gpu::BlurReduce<unsigned char, MAX_PYR_LEVELS, unsigned int>( dVars.uTmpPyr1, dVars.uTmp1, dVars.uTmp1 );

    // copy greyscale images
    KF.Image        = vImgs[0].Image;
    KF.ThumbImage   = cv::Mat( nThumbHeight, nThumbWidth, CV_8UC1 );
    dVars.uTmpPyr1[MAX_PYR_LEVELS-1].MemcpyToHost( KF.ThumbImage.data );

    gphp::GenerateThumbnail( vImgs[0].Image, KF.ThumbImage );



    //------------------------------------------- Depth Data

    // if image are disparities, convert to depth map
    if( g_bDisparityMaps ) {
        Eigen::Matrix3d             K = CamModel.K(0);
        const float                 fBaseline = CamModel.GetPose()( 1, 3 );

        gphp::Disp2Depth( vImgs[1].Image, K(0,0), fBaseline );
    }

    // second image is either an RGB image from stereo, or a depth map
    // if camera provides depth maps use it...
    if( g_bHaveDepth ) {
        dVars.fTmpPyr1[0].MemcpyFromHost( vImgs[1].Image.data );
    } else {
        // ... otherwise calculate depth map
        // do stuff, and eventually copy map to 'fTmpPyr1'
    }


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

    // copy depth map
    KF.Depth        = cv::Mat( nImgHeight, nImgWidth, CV_32FC1 );
    KF.ThumbDepth   = cv::Mat( nThumbHeight, nThumbWidth, CV_32FC1 );
    dVars.fTmpPyr1[0].MemcpyToHost( KF.Depth.data );
    dVars.fTmpPyr1[MAX_PYR_LEVELS-1].MemcpyToHost( KF.ThumbDepth.data );

    return KF;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Load keyframe files into memory (vector of keyframes)
void LoadKeyframesFromFile(
        GetPot*                         cl,                 //< Input: Command line arguments
        GpuVars_t&                      dVars,              //< Input: GPU Workspace
        std::vector< Keyframe_t >&      vKeyframes          //< Output: Vector of keyframes
        )
{
    // set up filereader for keyframes
    CameraDevice Cam;
    std::string sKeyframePFile = cl->follow( "", 1, "-kpfile" );
    std::string sKeyframeDir = cl->follow( "./Keyframes", 1, "-kdir" );
    std::string sKeyframeIFile = cl->follow( "Left.*", 1, "-kifile" );
    std::string sKeyframeDFile = cl->follow( "Depth.*", 1, "-kdfile" );
    Cam.SetProperty("DataSourceDir", sKeyframeDir );
    Cam.SetProperty("Channel-0", sKeyframeIFile );
    Cam.SetProperty("Channel-1", sKeyframeDFile );
    Cam.SetProperty("NumChannels", 2 );

    if( sKeyframePFile.empty() == true ) {
        std::cout << "-- Keyframe files not provided. No keyframes preloaded." << std::endl;
        return;
    }

    std::cout << "Loading keyframes..." << std::endl;

    if( g_bDisparityMaps == true ) {
        std::cout << "-- Reading images as disparities." << std::endl;
    }
    else {
        std::cout << "-- Reading images as depth maps." << std::endl;
    }

    // init driver
    if( !Cam.InitDriver( "FileReader" ) ) {
            std::cerr << "Invalid input device to load poses." << std::endl;
            exit(0);
    }

    // pose file
    std::ifstream pFile;
    pFile.open( sKeyframeDir + "/" + sKeyframePFile );
    if( pFile.is_open() == false ) {
        std::cerr << "Error opening keyframe pose file!" << std::endl;
        exit(-1);
    }

    // data containers
    Eigen::Vector6d                     Pose;
    std::vector< rpg::ImageWrapper >    vImgs;

    // iterate through pose file
    while( 1 ) {
        // read pose
        pFile >> Pose(0) >> Pose(1) >> Pose(2) >> Pose(3) >> Pose(4) >> Pose(5);

        if( pFile.eof( ) ) {
            break;
        }

        // read images
        if( Cam.Capture( vImgs ) == false ) {
            std::cerr << "Error capturing keyframe image." << std::endl;
            exit(-1);
        }

        vKeyframes.push_back( CreateKeyframe( dVars, vImgs[0].Image, vImgs[1].Image, mvl::Cart2T(Pose) ) );
    }

    pFile.close( );
    std::cout << "-- Loaded " << vKeyframes.size() << " keyframes from file." << std::endl;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Upload Keyframe data to GPU variables
void UploadKeyframe(
        const Keyframe_t&       Keyframe,         //< Input: Keyframe
        GpuVars_t&              dVars             //< Input/Output: GPU Workspace
        )
{
    // get GUI variables
    pangolin::Var<unsigned int>     ui_nBlur("ui.Blur");
    pangolin::Var<bool>             ui_bBilateralFiltDepth("ui.Cross Bilateral Filter (Depth)");
    pangolin::Var<int>              ui_nBilateralWinSize("ui.-- Size");
    pangolin::Var<float>            ui_gs("ui.-- Spatial");
    pangolin::Var<float>            ui_gr("ui.-- Depth Range");
    pangolin::Var<float>            ui_gc("ui.-- Color Range");

    // update keyframe image
    dVars.KeyGreyPyr[0].MemcpyFromHost( Keyframe.Image.data );

    for( int ii = 0; ii < ui_nBlur; ii++ ) {
        Gpu::Blur( dVars.KeyGreyPyr[0], dVars.uTmp1 );
    }
    Gpu::BlurReduce<unsigned char, MAX_PYR_LEVELS, unsigned int>( dVars.KeyGreyPyr, dVars.uTmp1, dVars.uTmp2 );


    // update keyframe depth map
    dVars.KeyDepthPyr[0].MemcpyFromHost( Keyframe.Depth.data );

    // cross-bilateral filter the downsampled depth maps
    Gpu::BoxReduce< float, MAX_PYR_LEVELS, float >( dVars.KeyDepthPyr );
    if( ui_bBilateralFiltDepth == true ) {
        dVars.fTmpPyr1[0].CopyFrom( dVars.KeyDepthPyr[0] );
        for(int ii = 1; ii < MAX_PYR_LEVELS; ii++ ) {
            Gpu::BilateralFilter<float,float,unsigned char>( dVars.fTmpPyr1[ii], dVars.KeyDepthPyr[ii], dVars.GreyPyr[ii],
                                                             ui_gs, ui_gr, ui_gc, ui_nBilateralWinSize );
        }
        dVars.KeyDepthPyr.CopyFrom(dVars.fTmpPyr1);
    }
}


*/

#endif // KEYFRAMES_H
