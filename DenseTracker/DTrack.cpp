#include <pangolin/pangolin.h>
#include <pangolin/glcuda.h>
#include <pangolin/glvbo.h>
#include <sophus/se3.hpp>
#include <kangaroo/kangaroo.h>
#include <kangaroo/../applications/common/ImageSelect.h>
#include <kangaroo/../applications/common/CameraModelPyramid.h>
#include <SceneGraph/SceneGraph.h>
#include <RPG/Devices.h>
#include <RPG/Utils/InitCam.h>
#include <RPG/Utils/InitIMU.h>
#include <Mvlpp/Mvl.h>
#include <boost/bind.hpp>

#include "Common.h"
#include "GpuHelpers.h"
#include "InitCamera.h"
#include "StreamHelpers.h"
#include "GLPath.h"
#include "GLPyrPath.h"
#include "Keyframes.h"

using namespace std;

//#define GROUND_TRUTH
//#define SENSOR_FUSION

#ifdef SENSOR_FUSION
#include <SensorFusion.h>

const unsigned int nImuFilterSize = 10;
Fusion::SensorFusion ImuFusion( nImuFilterSize );

#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void _HardReset( Eigen::Matrix4d* T_pc,
             Eigen::Matrix4d* T_wp,
             Eigen::Matrix4d* T_rv,
             bool* guiSetPrevious,
             GLPath* glPath
               )

{
    *T_pc = Eigen::Matrix4d::Identity();
    *T_wp = Eigen::Matrix4d( *T_rv );
    *guiSetPrevious = true;
    glPath->InitReset ();
    glPath->PushPose( *T_rv );
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Adjust mean and variance of Image1 brightness to be closer to Image2
inline void BrightnessCorrectionImagePair(
        unsigned char *pData1,
        unsigned char *pData2,
        int nImageSize
        )
{
    unsigned char* pData1_Orig = pData1;
    const int     nSampleStep = 1;
    int           nSamples    = 0;

     // compute mean
    float fMean1  = 0.0;
    float fMean2  = 0.0;
    float fMean12 = 0.0;
    float fMean22 = 0.0;

    for(int ii=0; ii<nImageSize; ii+=nSampleStep, pData1+=nSampleStep,pData2+=nSampleStep) {
        fMean1  += (*pData1);
        fMean12 += (*pData1) * (*pData1);
        fMean2  += (*pData2);
        fMean22 += (*pData2) * (*pData2);
        nSamples++;
    }

    fMean1  /= nSamples;
    fMean2  /= nSamples;
    fMean12 /= nSamples;
    fMean22 /= nSamples;

    // compute std
    float fStd1 = sqrt(fMean12 - fMean1*fMean1);
    float fStd2 = sqrt(fMean22 - fMean2*fMean2);

    // mean diff;
    //float mdiff = mean1 - mean2;
    // std factor
    float fRatio = fStd2/fStd1;
    // normalize image
    float tmp;
    // reset pointer
    pData1 = pData1_Orig;

    int nMean1 = (int)fMean1;
    int nMean2 = (int)fMean2;

    for(int ii=0; ii < nImageSize; ++ii) {

        tmp = (float)( pData1[ii] - nMean1 )*fRatio + nMean2;
        if(tmp < 0)  tmp = 0;
        if(tmp > 255) tmp = 255;
        pData1[ii] = (unsigned char)tmp;
    }

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void UnpackImages(
        vector< rpg::ImageWrapper >&    vImages,    //< Input:
        cv::Mat&                        Image,      //< Output:
        cv::Mat&                        Depth       //< Output:
    )
{
    //------------------------------------------------------------------------------
    // assuming vImages[0] is color-grey image

    //Image = vImages[0].Image;
    Image = vImages[0].Image.clone();

    // if image is RGB, convert to greyscale
    if( Image.channels() == 3 ) {
        cv::cvtColor( Image, Image, CV_RGB2GRAY, 1 );
    }

    //------------------------------------------------------------------------------
    // assuming vImages[1] is either depth/disp or another color-grey image

    Depth = vImages[1].Image.clone();

    // check if second image is type '8 unsigned char'
    if( Depth.type() == CV_8U ) {
        if( Depth.channels() == 3 ) {
            // second image is RGB, convert to greyscale
            cv::cvtColor( Depth, Depth, CV_RGB2GRAY, 1 );
        }
        if( Depth.channels() == 1 ) {
            // second image is greyscale, calculate depth map
            // EMPTY! =(
            // Depth = stuff
        }
    }

    // check if second image is type '16 unsigned char' -- given by Kinect
    if( vImages[1].Image.type() == CV_16U ) {
        // convert to float
        vImages[1].Image.convertTo( Depth, CV_32FC1 );
        Depth = Depth / g_fDepthScale;
    }
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void UploadImage(
        GpuVars_t&                                  dVars,              //< Input: GPU Workspace
        const cv::Mat&                              Image
        )
{
    pangolin::Var<unsigned int>     ui_nBlur("ui.Blur");

    // upload images
    dVars.GreyPyr[0].MemcpyFromHost( Image.data );

    // blur bottom image
    for( int ii = 0; ii < ui_nBlur; ii++ ) {
        Gpu::Blur( dVars.GreyPyr[0], dVars.uTmp1 );
    }

    // reduce and blur rest of pyramid
    Gpu::BlurReduce<unsigned char, MAX_PYR_LEVELS, unsigned int>( dVars.GreyPyr, dVars.uTmp1, dVars.uTmp2 );
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef SENSOR_FUSION
void imuNewData(const IMUData& data )
{
    ImuFusion.RegisterImuPose( data.accel(0), data.accel(1), data.accel(2), data.gyro(0), data.gyro(1), data.gyro(2),
                               data.timestamp_system, data.timestamp_system );
}
#endif



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Init
    InitPermutationMatrices();

    // initialize max number of iterations to perform at each pyramid level
    // level 0 is finest (ie. biggest image)
    g_vPyrMaxIters.resize( MAX_PYR_LEVELS );
    g_vPyrMaxIters.setZero();
    g_vPyrMaxIters << 2, 3, 4, 5, 5;
//    g_vPyrMaxIters << 1, 2, 3;



    // initialize if full estimate should be performed at a particular level
    // 1: full estimate          0: just rotation
    g_vPyrFullMask.resize( MAX_PYR_LEVELS );
    g_vPyrFullMask.setZero();
    g_vPyrFullMask << 1, 1, 1, 1, 0;
//    g_vPyrFullMask << 1, 1, 1;

    // initialize motion model
    g_vMotionModel << 0.3, 0.2, 0.05, 0.01, 0.01, 0.7;


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // init camera based on command line args
    GetPot cl( argc, argv );
    CameraDevice* pCam = InitCamera( &cl );

    // color to depth calibration.. if aligned, T_cd = I4
    Eigen::Matrix4d Tcd;
    if( g_bAligned == false ) {
        Tcd(0,4) = pCam->GetProperty<double>("Depth0Baseline", 0) / 100;
        std::cout << "Baseline: " << Tcd(0,4) << std::endl;
    }
    /*
    Tcd <<  0.9992665017642554, -0.00796822821816846, 0.03745618494842504, -0.007875168421226117,
            0.007999328442815506, 0.9999677733021591, -0.0006805156762577497, -0.03780141340682321,
            -0.03744955535505597, 0.0009796408448271114, 0.999298039179265, 0.02700363094655044,
            0, 0, 0, 1;
    /* */

    // read camera model file
    cout << "Loading camera model file..." << endl;
    CameraModelPyramid CamModel_I( pCam->GetProperty( "CamModFileName" ) );
    CameraModelPyramid CamModel_D( pCam->GetProperty( "CamModFileName" ) );
    CamModel_I.PopulatePyramid(MAX_PYR_LEVELS);
    CamModel_D.PopulatePyramid(MAX_PYR_LEVELS);
    cout << "... Done!" << endl;

#ifdef SENSOR_FUSION
    IMUDevice IMU;
    rpg::InitIMU( IMU, cl );
    IMU.RegisterIMUDataCallback( imuNewData );
#endif

    // vector of images captured
    vector< rpg::ImageWrapper > vImages;

    // reference to images used throughout this code
    // if a particular input device does not provide the format required, a conversion must be performed
    // the code expects: unsigned char for GreyImg
    //                   float         for DepthImg
    cv::Mat                     GreyImage;
    cv::Mat                     DepthImage;

    // initial capture for image properties
    // for the LIVE kinect this is a hack since the first images suck
    for( int ii = 0; ii < 5; ii++ ) {
        pCam->Capture( vImages );
    }
    UnpackImages( vImages, GreyImage, DepthImage );


    // image properties
    const unsigned int nImgWidth = GreyImage.cols;
    assert( nImgWidth == CamModel.Width() );
    const unsigned int nImgHeight = GreyImage.rows;
    assert( nImgHeight == CamModel.Height() );
    const unsigned int nThumbHeight = nImgHeight >> MAX_PYR_LEVELS-1;
    const unsigned int nThumbWidth = nImgWidth >> MAX_PYR_LEVELS-1;

    // GPU variable holder
    GpuVars_t   dVars( nImgHeight, nImgWidth );

    // GPU temporal workspace
//    gphp::Init( nImgWidth * nImgHeight * 5 );

    // convert disparity to depth map
    if( g_bDisparityMaps ) {
        Disp2Depth( dVars, DepthImage, CamModel_D.K()(0,0), CamModel_D.GetPose()( 1, 3 ) );
    }

    // print some info
    cout << "Initial Config ____________________________________________________" << endl;
    cout << "-- Image Width: " << CamModel_I.Width() << endl;
    cout << "-- Image Height: " << CamModel_I.Height() << endl;
    cout << "-- Thumbnails Width: " << nThumbWidth << endl;
    cout << "-- Thumbnails Height: " << nThumbHeight << endl;
    cout << "-- K Matrix: " << endl;
    cout << CamModel_I.K() << endl;


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // create a GUI window
    pangolin::CreateGlutWindowAndBind( "Dense Tracker", 1280, 720 );
    glewInit();
    cudaGLSetGLDevice(0);
    SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState glState( pangolin::ProjectionMatrix( 640, 480, 420, 420, 320, 240, 0.1, 1000 ),
                                         pangolin::ModelViewLookAt( -6, 0, -10, 1, 0, 0, pangolin::AxisNegZ ) );

    // Scenegraph to hold GLObjects and relative transformations
    SceneGraph::GLSceneGraph glGraph;

    // We define a new view which will reside within the container.
    pangolin::View glView3D;

    // We set the views location on screen and add a handler
    glView3D.SetHandler( new SceneGraph::HandlerSceneGraph( glGraph, glState, pangolin::AxisNone) );
    glView3D.SetDrawFunction( SceneGraph::ActivateDrawFunctor( glGraph, glState ) );
    glView3D.SetAspect( 640.0 / 480.0 );

    // side panel
    pangolin::CreatePanel("ui").SetBounds(0,1,0,pangolin::Attach::Pix(300));
    pangolin::Var<unsigned int>     ui_nRate("ui.Rate");
    pangolin::Var<float>            ui_fSqErr("ui.Mean Square Error");
    pangolin::Var<unsigned int>     ui_nImgIdx("ui.Image ID", 0);
    pangolin::Var<Eigen::Vector6d>  ui_CurPose("ui.Pose");
#ifdef GROUND_TRUTH
    pangolin::Var<Eigen::Vector6d>  ui_ErrorPose("ui.Error");
#endif
    pangolin::Var<unsigned int>     ui_nKeyIdx("ui.Key ID", 0);
    pangolin::Var<Eigen::Vector6d>  ui_KeyPose("ui.Key");
    pangolin::Var<Eigen::Vector6d>  ui_DeltaPose("ui.Delta");
    pangolin::Var<float>            ui_CenterPixelDepth("ui.Center Pixel Depth");
    pangolin::Var<int>              ui_nPyrLevel("ui.Pyramid Level",0,0,MAX_PYR_LEVELS-1);
    pangolin::Var<bool>             ui_bBrightCorrect("ui.Brightness Correct Images", true, true);
    pangolin::Var<bool>             ui_btnNewKeyframe("ui.New Keyframe",false,false);
    pangolin::Var<bool>             ui_bAutoKeyframes("ui.Auto Generate Keyframes", false, true);
    pangolin::Var<float>            ui_fKeyframePtsThreshold("ui.Auto Keyframe Pts Threshold",0.75,0,1);
    pangolin::Var<bool>             ui_bUseIMU("ui.UseIMU", true, true);
    pangolin::Var<bool>             ui_bRefineKeyframes("ui.Refine Keyframes", true, true);
    pangolin::Var<unsigned int>     ui_nNumRefinements("ui.Number of Refinements", 0);
    pangolin::Var<bool>             ui_btnRelocalize("ui.Relocalize",false,false);
    pangolin::Var<float>            ui_fRelocalizationRatio("ui.Reloc Luminance:Depth Ratio",1.0,0,1.0);
    pangolin::Var<bool>             ui_bAutoRelocalize("ui.Auto Relocalize", false, true);
    pangolin::Var<float>            ui_fRelocalizationErrorThreshold("ui.Auto Reloc Error Threshold",500,0,3000);
    pangolin::Var<bool>             ui_bBreakEarly("ui.Break Early", false, true);
    pangolin::Var<float>            ui_fBreakErrorThreshold("ui.Break Early Error Threshold",0.8,0,2);
    pangolin::Var<unsigned int>     ui_nNumIters("ui.Number of Iterations", 0);
    pangolin::Var<unsigned int>     ui_nBlur("ui.Blur",1,0,5);
    pangolin::Var<float>            ui_fNormC("ui.Norm C",10,0,100);
    pangolin::Var<bool>             ui_bDiscardMaxMin("ui.Discard Max-Min Pix Values", true, true);
    pangolin::Var<bool>             ui_bBilateralFiltDepth("ui.Cross Bilateral Filter (Depth)", false, true);
    pangolin::Var<int>              ui_nBilateralWinSize("ui.-- Size",5, 1, 20);
    pangolin::Var<float>            ui_gs("ui.-- Spatial",1, 1E-3, 5);
    pangolin::Var<float>            ui_gr("ui.-- Depth Range",0.5, 1E-3, 10);
    pangolin::Var<float>            ui_gc("ui.-- Color Range",10, 1E-3, 20);
    pangolin::Var<float>            ui_fMinPts("ui.Min Points Estimate Acceptance Threshold",0.33,0,1);
    pangolin::Var<bool>             ui_bSaveKeyframesOnExit("ui.Save Keyframes On Exit", false, true);


    // init image index of start frame provided through console
    unsigned int nStartFrame      = cl.follow( 0, 1, "-sf"  );
    ui_nImgIdx = nStartFrame;

#ifdef GROUND_TRUTH
    vector< Eigen::Vector6d >       vTruePoses;
    {
        // load ground truth poses if available
        string      sSourceDir       = cl.follow( ".", 1, "-sdir"  );
        string      sGroundTruthFile = cl.follow( "Poses.txt", 1, "-gt" );

        ifstream pFile;
        pFile.open( sSourceDir + "/" + sGroundTruthFile );
        if( pFile.is_open() == false ) {
            cerr << "Error opening ground truth pose file!" << endl;
        } else {

            Eigen::Vector6d                 Pose;

            while( true ) {
                // read pose
                pFile >> Pose(0) >> Pose(1) >> Pose(2) >> Pose(3) >> Pose(4) >> Pose(5);

                if( pFile.eof( ) ) {
                    break;
                }
                vTruePoses.push_back( Pose );
            }

            pFile.close( );
        }
    }
#endif

    // create a view container
    pangolin::View& guiContainer = pangolin::CreateDisplay()
            .SetBounds(0,1,pangolin::Attach::Pix(300),1)
            .SetLayout(pangolin::LayoutEqual);
    const unsigned int nNumConts = 4;
    for( unsigned int ii = 0; ii < nNumConts; ii ++ ) {
        pangolin::View& pangoView = pangolin::CreateDisplay();
        pangoView.SetAspect((double)nImgWidth/nImgHeight);
        guiContainer.AddDisplay(pangoView);
    }

    // add 3d view to container
    guiContainer.AddDisplay( glView3D );

    // draw grid on 3D window
    SceneGraph::GLGrid glGrid;
    glGraph.AddChild( &glGrid );

    // gl axis
    SceneGraph::GLAxis glCurPose;
    SceneGraph::GLAxis glPrevPose;
    SceneGraph::GLAxis glKeyPose;
    glPrevPose.SetScale( 2.0 );
    glKeyPose.SetScale( 3.0 );
    glGraph.AddChild( &glCurPose );
    glGraph.AddChild( &glPrevPose );
    glGraph.AddChild( &glKeyPose );


    // gl path
    GLPath glPath;
    GLPyrPath glPyrPath( MAX_PYR_LEVELS );
//    glGraph.AddChild( &glPath );
    glGraph.AddChild(&glPyrPath);

    // initialize vbo's
    vector < pangolin::GlBufferCudaPtr* >      vVBO;
    vVBO.resize( MAX_PYR_LEVELS );
    vector < pangolin::GlBufferCudaPtr* >      vCBO;
    vCBO.resize( MAX_PYR_LEVELS );
    vector < pangolin::GlBuffer* >      vIBO;
    vIBO.resize( MAX_PYR_LEVELS );
    vector < SceneGraph::GLObject* >           glVBO;
    glVBO.resize( MAX_PYR_LEVELS );

    for( int ii = 0; ii < MAX_PYR_LEVELS; ii++ ) {
        const unsigned              PyrLvlWidth = nImgWidth >> ii;
        const unsigned              PyrLvlHeight = nImgHeight >> ii;

        vVBO[ii] = new pangolin::GlBufferCudaPtr( pangolin::GlArrayBuffer, PyrLvlWidth*PyrLvlHeight, GL_FLOAT, 4,
                                       cudaGraphicsMapFlagsWriteDiscard, GL_STREAM_DRAW );
        vCBO[ii] = new pangolin::GlBufferCudaPtr( pangolin::GlArrayBuffer, PyrLvlWidth*PyrLvlHeight, GL_UNSIGNED_BYTE, 4,
                                       cudaGraphicsMapFlagsWriteDiscard, GL_STREAM_DRAW );
        vIBO[ii] = new pangolin::GlBuffer();
        pangolin::MakeTriangleStripIboForVbo(*vIBO[ii], PyrLvlWidth, PyrLvlHeight );

        // add vbo to scenegraph
        glVBO[ii] = new SceneGraph::GLVbo( vVBO[ii], vIBO[ii], vCBO[ii] );
        glKeyPose.AddChild( glVBO[ii] );
    }

    pangolin::ActivateDrawPyramid< unsigned char, MAX_PYR_LEVELS >      DrawImg( dVars.GreyPyr, GL_LUMINANCE8, false, true );
    pangolin::ActivateDrawPyramid< unsigned char, MAX_PYR_LEVELS >      DrawKeyImg( dVars.KeyGreyPyr, GL_LUMINANCE8, false, true );
    pangolin::ActivateDrawImage< float4 >                               DrawDebugImg( dVars.Debug, GL_RGBA32F_ARB, false, true );
    pangolin::ActivateDrawPyramid< float, MAX_PYR_LEVELS >              DrawDepth( dVars.KeyDepthPyrNormalized, GL_LUMINANCE32F_ARB, false, true );

    // add images to the container
    guiContainer[0].SetDrawFunction(boost::ref( DrawImg ));
    guiContainer[1].SetDrawFunction(boost::ref( DrawKeyImg ));
    guiContainer[2].SetDrawFunction(boost::ref( DrawDebugImg ));
    guiContainer[3].SetDrawFunction(boost::ref( DrawDepth ));


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // for framerate calculations
    double dT;

    // Pose variables
    Eigen::Matrix4d T_wk;                   // keyframe pose in world reference frame
    Eigen::Matrix4d T_wp;                   // previous pose in world reference frame
    Eigen::Matrix4d T_wc;                   // current pose in world reference frame
    Eigen::Matrix4d T_kc;                   // current pose relative to keyframe
    Eigen::Matrix4d T_pc;                   // current pose relative to previous pose
    Eigen::Matrix4d T_pc_prev;              // previous pose update (use if motion model is enabled)

    // generate thumbnail
    cv::Mat ThumbImage( nThumbHeight, nThumbWidth, CV_8UC1 );
    cv::Mat ThumbDepth( nThumbHeight, nThumbWidth, CV_32FC1 );
    GenerateThumbnail( dVars, GreyImage, ThumbImage );
    GenerateDepthThumbnail( dVars, DepthImage, ThumbDepth );

    // vector of keyframes
    vector< Keyframe_t >        vKeyframes;

    // preload keyframes (if available)
    LoadKeyframesFromFile( &cl, dVars, vKeyframes );

    // keyframe index
    unsigned int    nKeyIdx;

    // initial keyframe
    float   fBestKeyScore;
    if( vKeyframes.size() == 0 ) {
        cout << "Creating first keyframe from current images...";
        nKeyIdx = 0;
        vKeyframes.push_back( CreateKeyframe( dVars, GreyImage, DepthImage, Eigen::Matrix4d::Identity() ) );
        cout << " done!" << endl;
    } else {
        cout << "Estimating initial keyframe..." << endl;
        dT = mvl::Tic();
        nKeyIdx = FindBestKeyframe( vKeyframes, ThumbImage, ThumbDepth );
        fBestKeyScore = vKeyframes[nKeyIdx].ThumbScore;
        cout << "-- Best guess was # " << nKeyIdx << ". ( " << mvl::TocMS(dT) << " ms )" << endl;
    }

    // normalize greyscale image to keyframe
    if( ui_bBrightCorrect == true ) {
        BrightnessCorrectionImagePair( GreyImage.data, vKeyframes[nKeyIdx].Image.data, nImgHeight * nImgWidth );
    }

    // upload grey image to GPU
    UploadImage( dVars, GreyImage );

    // initialize poses
    T_wk = /*T_wr*/ vKeyframes[nKeyIdx].Pose * g_Trv;
    T_kc = Eigen::Matrix4d::Identity();
    T_wc = T_wk;
    T_pc = Eigen::Matrix4d::Identity();
    T_wp = T_wc;


#ifdef SENSOR_FUSION
    ImuFusion.ResetCurrentPose( mvl::T2Cart( T_wc ), Eigen::Vector3d::Zero() , Eigen::Vector2d::Zero() );
#endif


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // control variables
    bool            guiRunning      = false;
    bool            guiSetKeyframe  = false;
    bool            guiGo           = false;
    bool            bNewCapture     = false;
    bool            bShowKeyframes  = true;


    // keyboard callbacks
    pangolin::RegisterKeyPressCallback(' ', [&guiRunning](){ guiRunning = !guiRunning; });
    pangolin::RegisterKeyPressCallback('k', [&guiSetKeyframe](){ guiSetKeyframe = true; });
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + GLUT_KEY_RIGHT, [&guiGo](){ guiGo = true; });
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'r', boost::bind( _HardReset, &T_kc, &T_wk, &g_Trv, &guiSetKeyframe, &glPath ) );
    pangolin::RegisterKeyPressCallback('r',[&T_kc](){ T_kc = Eigen::Matrix4d::Identity(); });
    pangolin::RegisterKeyPressCallback('~', [&guiContainer](){ static bool showpanel = true; showpanel = !showpanel;
                                        if(showpanel) { guiContainer.SetBounds(0,1,pangolin::Attach::Pix(300), 1); } else
                                        { guiContainer.SetBounds(0,1,0, 1); } pangolin::Display("ui").Show(showpanel); } );
    pangolin::RegisterKeyPressCallback('1', [&guiContainer](){ guiContainer[0].ToggleShow(); });
    pangolin::RegisterKeyPressCallback('2', [&guiContainer](){ guiContainer[1].ToggleShow(); });
    pangolin::RegisterKeyPressCallback('3', [&guiContainer](){ guiContainer[2].ToggleShow(); });
    pangolin::RegisterKeyPressCallback('4', [&guiContainer](){ guiContainer[3].ToggleShow(); });
    pangolin::RegisterKeyPressCallback('5', [&guiContainer](){ guiContainer[4].ToggleShow(); });
    pangolin::RegisterKeyPressCallback('6', [&bShowKeyframes](){ bShowKeyframes = !bShowKeyframes; });
    pangolin::RegisterKeyPressCallback('7', [&glCurPose,&glPrevPose,&glKeyPose](){ glCurPose.SetVisible( !glCurPose.IsVisible() );
                                        glPrevPose.SetVisible( !glPrevPose.IsVisible() ); glKeyPose.SetVisible( !glKeyPose.IsVisible() ); });


    // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Main Loop
    //
    double          dLError = 0;            // localization mean square error
    unsigned int    nLObs   = 0;            // localization number of observations
    while( !pangolin::ShouldQuit() ) {

        // rate counter
        dT = mvl::Tic();

        // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Initialize....
        //

        T_wc = T_wp;

#ifdef SENSOR_FUSION
        // Use IMU to seed initial estimate
        T_wc = mvl::Cart2T( ImuFusion.GetCurrentPose().m_dPose );
#endif

        // Base on IMU's estimate, find closest keyframe and do ESM
        nKeyIdx = FindClosestKeyframe( vKeyframes, T_wc );
        glPyrPath.InitReset();

        unsigned int    nIters = 0;         // number of iterations used to compute estimate
        for( unsigned int nNumRefinements = 1; nNumRefinements < 10; nNumRefinements++ ) {

            // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Load Keyframe
            //

            // upload keyframe to GPU
            UploadKeyframe( vKeyframes[nKeyIdx], dVars );

            // update keyframe's pose (in vision frame)
            T_wk = vKeyframes[nKeyIdx].Pose * g_Trv;

            // initialize T_kc from our current global position and that of the keyframe
            T_kc = mvl::TInv( T_wk ) * T_wc;


            // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Generate VBO
            //

            const unsigned              CurPyrLvlW = nImgWidth >> ui_nPyrLevel;
            const unsigned              CurPyrLvlH = nImgHeight >> ui_nPyrLevel;

            // Update (VBO) Vertex Buffer Object
            {
                Eigen::Matrix3d                 K = CamModel_D.K( ui_nPyrLevel );
                pangolin::CudaScopedMappedPtr   var( *(vVBO[ ui_nPyrLevel]) );
                Gpu::Image< float4 >            dVbo( (float4*)*var, CurPyrLvlW, CurPyrLvlH );
                Gpu::DepthToVbo( dVbo, dVars.KeyDepthPyr[ ui_nPyrLevel ], K( 0, 0 ), K( 1, 1 ), K( 0, 2 ), K( 1, 2 ) );
            }

            // Update (CBO) Colored Buffered Object
            {
                pangolin::CudaScopedMappedPtr var( *(vCBO[ ui_nPyrLevel]) );
                Gpu::Image< uchar4 >          dCbo( (uchar4*)*var, CurPyrLvlW, CurPyrLvlH );
                Gpu::ConvertImage< uchar4, unsigned char >( dCbo, dVars.KeyGreyPyr[ ui_nPyrLevel ] );
            }

            // set all VBOs invisible
            for( int ii = 0; ii < MAX_PYR_LEVELS; ii++ ) {
                glVBO[ii]->SetVisible(false);
            }
            glVBO[ ui_nPyrLevel ]->SetVisible(true);



            // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Localization
            //

            for( int PyrLvl = MAX_PYR_LEVELS-1; PyrLvl >= 0; PyrLvl-- ) {
                dLError = 0;
                for(int ii = 0; ii < g_vPyrMaxIters[PyrLvl]; ii++ ) {
                    const unsigned              PyrLvlWidth = nImgWidth >> PyrLvl;
                    const unsigned              PyrLvlHeight = nImgHeight >> PyrLvl;

                    Eigen::Matrix3d             K_I = CamModel_I.K( PyrLvl );
                    Eigen::Matrix3d             K_D = CamModel_D.K( PyrLvl );
                    Sophus::SE3d                sT_kc = Sophus::SE3d( T_kc );
                    Sophus::SE3d                sTcd = Sophus::SE3d( Tcd );
                    Eigen::Matrix<double,3,4>   KTck = K_I * sT_kc.inverse().matrix3x4();

                    float fNormC = ui_fNormC * (1 << PyrLvl );

                    // build system
                    Gpu::LeastSquaresSystem<float,6> LSS = Gpu::PoseRefinementFromDepthESM( dVars.GreyPyr[PyrLvl], dVars.KeyGreyPyr[PyrLvl],
                                                                                            dVars.KeyDepthPyr[PyrLvl], sTcd.matrix3x4(), KTck, fNormC,
                                                                                            K_D(0,0), K_D(1,1), K_D(0,2), K_D(1,2), dVars.Workspace,
                                                                                            dVars.Debug.SubImage(PyrLvlWidth, PyrLvlHeight),
                                                                                            ui_bDiscardMaxMin, 0.3, 20.0 );

                    Eigen::Matrix<double,6,6>   LHS = LSS.JTJ;
                    Eigen::Vector6d             RHS = LSS.JTy;

                    // solve system
                    Eigen::Vector6d             X;

                    // check if we are solving only for rotation, or full estimate
                    if( g_vPyrFullMask(PyrLvl) != 0 ) {
                        Eigen::FullPivLU<Eigen::Matrix<double,6,6> >    lu_JTJ(LHS);

                        // check degenerate system
                        if( lu_JTJ.rank() < 6 ) {
                            cerr << "warning(@L" << PyrLvl+1 << "I" << ii+1 << ") LS trashed. " << "Rank: " << lu_JTJ.rank() << endl;
                            continue;
                        }

                        X = - (lu_JTJ.solve(RHS));
                    } else {
                        // extract rotation information only
                        Eigen::Matrix<double,3,3>                       rLHS = LHS.block<3,3>(3,3);
                        Eigen::Vector3d                                 rRHS = RHS.tail(3);
                        Eigen::FullPivLU< Eigen::Matrix<double,3,3> >   lu_JTJ(rLHS);

                        // check degenerate system
                        if( lu_JTJ.rank() < 3 ) {
                            cerr << "warning(@L" << PyrLvl+1 << "I" << ii+1 << ") LS trashed. " << "Rank: " << lu_JTJ.rank() << endl;
                            continue;
                        }

                        Eigen::Vector3d rX;
                        rX = - (lu_JTJ.solve(rRHS));

                        // pack solution
                        X.setZero();
                        X.tail(3) = rX;
                    }

                    // if we have too few observations, discard estimate
                    if( (float)LSS.obs < ui_fMinPts * (float)( PyrLvlWidth * PyrLvlHeight ) ) {
                        cerr << "warning(@L" << PyrLvl+1 << "I" << ii+1 << ") LS trashed. " << "Too few pixels!" << endl;
                        continue;
                    }

                    // everything seems fine... apply update
                    T_kc = (T_kc.inverse() * Sophus::SE3::exp(X).matrix()).inverse();

                    // store update on GLObject
                    glPyrPath.PushPose( PyrLvl, T_wk * T_kc * g_Tvr );

                    double dNewError = LSS.sqErr / LSS.obs;

                    // only show error of last level so the GUI doesn't go too crazy
                    if( PyrLvl == 0 ) {
                        nLObs = LSS.obs;
                        ui_fSqErr =  dNewError;
                    }

                    // if error decreases too slowly, break out of this level
                    if( ( fabs( dNewError - dLError ) < ui_fBreakErrorThreshold ) && ui_bBreakEarly ) {
                        dLError = dNewError;
                        break;
                    }

                    // update error
                    dLError = dNewError;

                    // increment number of iterations
                    nIters ++;
                }
            }

            // update number of keyframe refinement steps
            ui_nNumRefinements = nNumRefinements;

            // update pose estimate
            T_wc = T_wk * T_kc;

            // find closest keyframe given this new pose
            unsigned int nNewKeyframe = FindClosestKeyframe( vKeyframes, T_wc );

            // if the keyframe is the same we just localized against, then break
            if( nNewKeyframe == nKeyIdx || !ui_bRefineKeyframes ) {
                break;
            }

            // ... otherwise, keep going!
            nKeyIdx = nNewKeyframe;
        }

        // calculate new T_pc
        T_pc = mvl::TInv(T_wp) * T_wc;


        // relocalize if error is too high or if requested by user
        if( ( (ui_fSqErr > ui_fRelocalizationErrorThreshold) && ui_bAutoRelocalize ) || pangolin::Pushed( ui_btnRelocalize ) ) {
            cerr << "warning: I think I am lost! Calling relocalizer..." << endl;

            // reset any image processing done before
            UnpackImages( vImages, GreyImage, DepthImage );

            // load thumbnail
//            gphp::GenerateThumbnail( GreyImg, ThumbImage );
            GenerateThumbnail( dVars, GreyImage, ThumbImage );
            GenerateDepthThumbnail( dVars, DepthImage, ThumbDepth );

            // call relocalizer!
            nKeyIdx = FindBestKeyframe( vKeyframes, ThumbImage, ThumbDepth );
            fBestKeyScore = vKeyframes[nKeyIdx].ThumbScore;
            cerr << "-- Best guess was # " << nKeyIdx << "." << endl;

            // initialize pose with best keyframe
            T_wk = /*T_wr*/ vKeyframes[nKeyIdx].Pose * g_Trv;
            T_kc = Eigen::Matrix4d::Identity();
            T_wc = T_wk;
            T_pc = Eigen::Matrix4d::Identity();
            T_wp = T_wc;
        }


        // create new keyframe is matching points are too few or if requested by user
        if( ( (nLObs < ui_fKeyframePtsThreshold * (nImgHeight*nImgWidth)) && ui_bAutoKeyframes ) || pangolin::Pushed( ui_btnNewKeyframe ) ) {
            nKeyIdx = vKeyframes.size();
            cout << "Creating new keyframe (#" << nKeyIdx << ")." << endl;
            UnpackImages( vImages, GreyImage, DepthImage );
            if( g_bDisparityMaps ) {
                Disp2Depth( dVars, DepthImage, CamModel_D.K()(0,0), CamModel_D.GetPose()( 1, 3 ) );
            }
            vKeyframes.push_back( CreateKeyframe( dVars, GreyImage, DepthImage,
                                                  mvl::Cart2T(mvl::T2Cart(T_wc)) * g_Tvr ) );
            T_wk = T_wc;
            T_kc = Eigen::Matrix4d::Identity();
            T_wc = T_wk;
            T_pc = Eigen::Matrix4d::Identity();
            T_wp = T_wc;
        }

//        cv::imshow("TN",vKeyframes[nKeyIdx].ThumbImage);
//        cv::imshow("TN",ThumbImage);
//        cv::waitKey(1);


        // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Capture
        //
        if( guiRunning || pangolin::Pushed(guiGo) ) {

            // accept estimate
            T_wp = T_wc;
            T_pc = Eigen::Matrix4d::Identity();

#ifdef SENSOR_FUSION
            ImuFusion.RegisterGlobalPose( mvl::T2Cart(T_wc), vImages[0].Map.GetProperty<double>("LoggerTime"),
                                          vImages[0].Map.GetProperty<double>("LoggerTime") );
#endif


            // capture an image
            if( pCam->Capture( vImages ) == false ) {
                cerr << "Error capturing images. Did I ran out of them?" << endl;
                exit(0);
            }

            UnpackImages( vImages, GreyImage, DepthImage );

            // normalize greyscale image to keyframe
            if( ui_bBrightCorrect == true ) {
                BrightnessCorrectionImagePair( GreyImage.data, vKeyframes[nKeyIdx].Image.data, nImgHeight * nImgWidth );
            }

            if( g_bDisparityMaps ) {
                Disp2Depth( dVars, DepthImage, CamModel_D.K()(0,0), CamModel_D.GetPose()( 1, 3 ) );
            }

            unsigned int Idx = ui_nImgIdx;
            Idx++;
            ui_nImgIdx = Idx;
            bNewCapture = true;
        }

        // upload greyscale image to GPU
        UploadImage( dVars, GreyImage );


        ///------------------------------------------------------------------------------------------------------------


        // update and render stuff
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
        glColor4f( 1, 1, 1, 1);

        glState.Follow( T_wc );

        // render keyframes
        if( bShowKeyframes ) {
            glView3D.Activate( glState );
            RenderKeyframes( vKeyframes, fBestKeyScore * 2.0 );
        }

        // display rate
        ui_nRate = 1.0 / mvl::Toc(dT);

#ifdef GROUND_TRUTH
        if( ui_nImgIdx < vTruePoses.size() ) {
            ui_ErrorPose = mvl::T2Cart( (T_wc * g_Tvr)) - vTruePoses[ui_nImgIdx];
        }
#endif

        // update number of iterations
        ui_nNumIters = nIters;

        // current keyframe index
        ui_nKeyIdx = nKeyIdx;

        // display center pixel's depth info
        ui_CenterPixelDepth = vKeyframes[nKeyIdx].Depth.at<float>( nImgHeight/2, nImgWidth/2 );

        // draw left image
        DrawImg.SetLevel(ui_nPyrLevel);

        // draw key image
        DrawKeyImg.SetLevel(ui_nPyrLevel);

        // normalize and draw disparity
        dVars.KeyDepthPyrNormalized[ui_nPyrLevel].CopyFrom(dVars.KeyDepthPyr[ui_nPyrLevel]);
        float fMaxDepth = *max_element( vKeyframes[nKeyIdx].Depth.begin<float>(), vKeyframes[nKeyIdx].Depth.end<float>() );
        fMaxDepth = fMaxDepth > 20? 20 : fMaxDepth;
        nppiDivC_32f_C1IR( fMaxDepth, dVars.KeyDepthPyrNormalized[ui_nPyrLevel].ptr, dVars.KeyDepthPyrNormalized[ui_nPyrLevel].pitch,
                           dVars.KeyDepthPyrNormalized[ui_nPyrLevel].Size() );
        DrawDepth.SetLevel(ui_nPyrLevel);

        // update poses
        glCurPose.SetPose( T_wc );
        glPrevPose.SetPose( T_wp );
        glKeyPose.SetPose( T_wk );
        ui_CurPose = mvl::T2Cart( T_wc * g_Tvr );
        ui_KeyPose = mvl::T2Cart( T_wk * g_Tvr );
        ui_DeltaPose = mvl::T2Cart( g_Trv * T_pc * g_Tvr );

        pangolin::FinishGlutFrame();
    }

    // save keyframe poses to a file
    if( ui_bSaveKeyframesOnExit == true ) {

        std::ofstream       pFile;
        pFile.open( "SavedKeyframes/Keyframes.txt", std::ios::out | std::ios::app );
        pFile.precision( std::numeric_limits<double>::digits10 );
        std::string     Dir = "SavedKeyframes/";
        char            Index[10];
        for( int ii = 0; ii < vKeyframes.size(); ii++ ) {
            pFile << mvl::T2Cart( vKeyframes[ii].Pose ).transpose() << std::endl;
            /*
            sprintf( Index, "%05d", ii );
            std::string ImgFile;
            ImgFile = Dir + "Image_" + Index + ".pgm";
            cv::imwrite( ImgFile, vKeyframes[ii].Image );
            std::string DepthFile;
            DepthFile = Dir + "Depth_" + Index + ".pdm";
            ofstream pDFile( DepthFile.c_str(), ios::out | ios::binary );
            pDFile << "P7" << std::endl;
            pDFile << vKeyframes[ii].Depth.cols << " " << vKeyframes[ii].Depth.rows << std::endl;
            unsigned int Size = vKeyframes[ii].Depth.elemSize1() * vKeyframes[ii].Depth.rows * vKeyframes[ii].Depth.cols;
            pDFile << 4294967295 << std::endl;
            pDFile.write( (const char*)vKeyframes[ii].Depth.data, Size );
            pDFile.close();
            */
        }
        pFile.close();
    }

return 0;
}
