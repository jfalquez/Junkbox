#ifndef KEYFRAMES_H
#define KEYFRAMES_H

#include <kangaroo/kangaroo.h>
#include <kangaroo/../applications/common/CameraModelPyramid.h>
#include <opencv.hpp>

#include "Common.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Keyframe
struct Keyframe_t {
    Eigen::Matrix4d     Pose;
    cv::Mat             Image;
    cv::Mat             Depth;
    cv::Mat             ThumbImage;
    cv::Mat             ThumbDepth;
    float               Score;
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
        if( vKeyframes[ii].Score < fThreshold ) {
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
unsigned int FindBestKeyframe(
        std::vector< Keyframe_t >&          vKeyframes,         //< Input: Vector of keyframes
        const cv::Mat&                      Image,              //< Input: Decimated greyscale image
        unsigned int                        nKeyIdx = 0         //< Input: Keyframe ID hint.
        )
{
    // assume that if hint is 0, no hint was given
    if( nKeyIdx == 0 ) {

    } else {

    }

    unsigned int nBestIdx = 0;
    vKeyframes[0].Score = ScoreImages(Image, vKeyframes[0].ThumbImage);
    float fBestScore = vKeyframes[0].Score;
    for( int ii = 1; ii < vKeyframes.size(); ii++ ) {
        vKeyframes[ii].Score = ScoreImages(Image, vKeyframes[ii].ThumbImage);
        if( vKeyframes[ii].Score < fBestScore ) {
            fBestScore = vKeyframes[ii].Score;
            nBestIdx = ii;
        }
    }

//    std::cout << "Best Keyframe: " << nBestIdx << std::endl;
    return nBestIdx;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Load keyframe files into memory (vector of keyframes)
void LoadKeyframes(
        GetPot*                         cl,                 //< Input: Command line arguments
        std::vector< Keyframe_t >&      vKeyframes          //< Output: Vector of keyframes
        )
{
    std::cout << "Loading keyframes..." << std::endl;

    // set up filereader for keyframes
    CameraDevice Cam;
    std::string sKeyframeDir = cl->follow( "./Keyframes", 1, "-kdir" );
    std::string sKeyframePFile = cl->follow( "Keyframes.txt", 1, "-kpfile" );
    std::string sKeyframeIFile = cl->follow( "Left.*", 1, "-kifile" );
    std::string sKeyframeDFile = cl->follow( "Depth.*", 1, "-kdfile" );
    Cam.SetProperty("DataSourceDir", sKeyframeDir );
    Cam.SetProperty("Channel-0", sKeyframeIFile );
    Cam.SetProperty("Channel-1", sKeyframeDFile );
    Cam.SetProperty("NumChannels", 2 );

    // get camera model file
    std::string sSourceDir      = cl->follow( ".", 1, "-sdir"  );
    std::string sCamModFileName = cl->follow( "rcmod.xml", 1, "-rcmod" );
    CameraModelPyramid CamModel( sSourceDir + "/" + sCamModFileName );
    CamModel.PopulatePyramid(MAX_PYR_LEVELS);

    const unsigned int nImgHeight = CamModel.Height();
    const unsigned int nImgWidth = CamModel.Width();
    const unsigned int nThumbHeight = nImgHeight >> MAX_PYR_LEVELS-1;
    const unsigned int nThumbWidth = nImgWidth >> MAX_PYR_LEVELS-1;


    // check if -disp option was specified
    // this means that the keyframe images are disparities instead of depth maps
    bool     bDisparity;
    bDisparity = cl->search( "-disp" );

    if( bDisparity == true ) {
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

    // GPU storage for decimated depth map
    Gpu::Pyramid< unsigned char, MAX_PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage >   dGreyPyr( nImgWidth, nImgHeight );
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage >                     dBlurTmp1( nImgWidth, nImgHeight );
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage >                     dBlurTmp2( nImgWidth, nImgHeight );
    Gpu::Pyramid< float, MAX_PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage >           dDepthPyr( nImgWidth, nImgHeight );
    Gpu::Pyramid< float, MAX_PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage >           dDepthPyrTmp( nImgWidth, nImgHeight );

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


        // store data
        Keyframe_t PN;
        PN.Pose = mvl::Cart2T(Pose);

        // store images on GPU
        dGreyPyr[0].MemcpyFromHost( vImgs[0].Image.data );
        dDepthPyr[0].MemcpyFromHost( vImgs[1].Image.data );

        // downsample greyscale image
        Gpu::BlurReduce<unsigned char, MAX_PYR_LEVELS, unsigned int>( dGreyPyr, dBlurTmp1, dBlurTmp2 );

        // copy greyscale images
        PN.Image        = vImgs[0].Image;
        PN.ThumbImage   = cv::Mat( nThumbHeight, nThumbWidth, CV_32FC1 );
        dGreyPyr[MAX_PYR_LEVELS-1].MemcpyToHost( PN.ThumbImage.data );

        // if image are disparities, convert to depth map
        if( bDisparity ) {
            Eigen::Matrix3d             K = CamModel.K(0);
            const float                 fBaseline = CamModel.GetPose()( 1, 3 );

            Gpu::Disp2Depth( dDepthPyr[0], dDepthPyr[0], K(0,0), fBaseline );
        }

        // downsample depth map
        Gpu::BoxReduce< float, MAX_PYR_LEVELS, float >( dDepthPyr );


        // cross-bilateral filter the downsampled depth maps
        if( g_bBiFilterThumbs == true ) {
            dDepthPyrTmp[0].CopyFrom( dDepthPyr[0] );
            for(int ii = 1; ii < MAX_PYR_LEVELS; ii++ ) {
                Gpu::BilateralFilter< float, float, unsigned char >( dDepthPyrTmp[ii], dDepthPyr[ii], dGreyPyr[ii],
                                                                   g_dThumbFiltS, g_dThumbFiltD, g_dThumbFiltC, g_nThumbFiltSize );
            }
            dDepthPyr.CopyFrom( dDepthPyrTmp );
        }

        // copy depth map
        PN.Depth        = cv::Mat( nImgHeight, nImgWidth, CV_32FC1 );
        PN.ThumbDepth   = cv::Mat( nThumbHeight, nThumbWidth, CV_32FC1 );
        dDepthPyr[0].MemcpyToHost( PN.Depth.data );
        dDepthPyr[MAX_PYR_LEVELS-1].MemcpyToHost( PN.ThumbDepth.data );

        vKeyframes.push_back(PN);
    }

    pFile.close( );
    std::cout << "-- Loaded " << vKeyframes.size() << " keyframes from file." << std::endl;
}


#endif // KEYFRAMES_H
