#include <pangolin/pangolin.h>
#include <pangolin/glcuda.h>
#include <kangaroo/kangaroo.h>
#include <sophus/se3.h>
#include <kangaroo/../applications/common/ImageSelect.h>
#include <kangaroo/../applications/common/CameraModelPyramid.h>
#include <SceneGraph/SceneGraph.h>
#include <RPG/Devices.h>
#include <Mvlpp/Mvl.h>
#include "ParseArgs.h"
#include <boost/bind.hpp>
#include <CVars/CVar.h>
#include "CVarHelpers.h"
#include "GLPath.h"

using namespace std;

#define     SAVE_POSES      0
#define     USE_PRELOADED   0
#define     DEPTH_MAP       1

const int MAX_LEVELS = 5;

/// CVars
Eigen::Matrix<int,1,Eigen::Dynamic>& g_vPyrCycles = CVarUtils::CreateCVar( "Tracker.PyrCycles", Eigen::Matrix<int,1,Eigen::Dynamic>(), "Number of cycles per pyramid level." );
Eigen::Matrix<int,1,Eigen::Dynamic>& g_vPyrFullMask = CVarUtils::CreateCVar( "Tracker.PyrFullMask", Eigen::Matrix<int,1,Eigen::Dynamic>(), "Set 1 for full estimate, 0 for rotation only estimates." );
Eigen::Vector6d& g_vMotionModel = CVarUtils::CreateCVar( "Tracker.MotionModel", Eigen::Vector6d(), "Motion model used to discard bad estimates." );
unsigned int& g_nPoseDisplay = CVarUtils::CreateCVar("Gui.PoseDisplay", 5u, "Number of axis to draw for poses." );

std::ostream& operator<< (std::ostream& os, const Eigen::Vector6d& v)
{
    os << "( " << fixed << setprecision(2) << showpos << v(0) << ", " << v(1) << ", " << v(2) << ", " << v(3) << ", " << v(4) << ", " << v(5) << " )";
    return os;
}

std::istream& operator>> (std::istream& is, Eigen::Vector6d& v)
{
  is >> v(0);
  is >> v(1);
  is >> v(2);
  is >> v(3);
  is >> v(4);
  is >> v(5);
  return is;
}

#if USE_PRELOADED
struct PNode {
    Eigen::Vector6d RelPose;
    Eigen::Vector6d AbsPose;
    cv::Mat         Image;
    cv::Mat         Disparity;
};
#endif

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

int main(int argc, char** argv)
{
    // iitialize some CVars
    // initialize number of iterations to perform at each pyramid level
    // level 0 is finest (ie. biggest image)
    g_vPyrCycles.resize( MAX_LEVELS );
    g_vPyrCycles << 1, 2, 3, 4, 5;
    g_vPyrFullMask.resize( MAX_LEVELS );
    g_vPyrFullMask << 1, 1, 1, 1, 0;
    g_vMotionModel << 0.3, 0.2, 0.05, 0.01, 0.01, 0.7;

    // parse parameters
    CameraDevice* pCam = ParseArgs( argc, argv );

    // read camera model file
    std::string sCamModFileName = pCam->GetProperty( "CamModelFile", "rcmod.xml" );
    CameraModelPyramid CamModel( sCamModFileName );
    CamModel.PopulatePyramid(MAX_LEVELS);

    // vector of images captured
    vector< rpg::ImageWrapper > vImages;

    // initial capture for image properties
    pCam->Capture( vImages );

    // check if we have disparity image
    if( vImages.size() != 3 ) {
        cout << "Disparity image is required!" << endl;
        exit(-1);
    }

    // image properties
    const unsigned int nImgWidth = vImages[1].Image.cols;
    const unsigned int nImgHeight = vImages[0].Image.rows;

#if DEPTH_MAP
{
    Eigen::Matrix3d             K = CamModel.K(0);
    const float                 fBaseline = CamModel.GetPose()( 1, 3 );
    for( int ii = 0; ii < nImgHeight; ii++ ) {
        for( int jj = 0; jj < nImgWidth; jj++ ) {
            vImages[2].Image.at<float>(ii,jj) = K(0,0) * fBaseline / vImages[2].Image.at<float>(ii,jj);
        }
    }
}
#endif

    // create a GUI window
    pangolin::CreateGlutWindowAndBind("Dense Tracker",1280,720);
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
    glView3D.SetHandler( new SceneGraph::HandlerSceneGraph( glGraph, glState, pangolin::AxisNegZ ) );
    glView3D.SetDrawFunction( SceneGraph::ActivateDrawFunctor( glGraph, glState ) );

    // create a side panel
    pangolin::CreatePanel("ui").SetBounds(0,1,0,pangolin::Attach::Pix(300));
    pangolin::Var<float>            ui_fSqErr("ui.Mean Square Error");
    pangolin::Var<unsigned int>     ui_nImgIdx("ui.Image ID", 0);
    pangolin::Var<Eigen::Vector6d>  ui_CurPose("ui.Pose");
    pangolin::Var<unsigned int>     ui_nKeyIdx("ui.Key ID", 0);
    pangolin::Var<Eigen::Vector6d>  ui_KeyPose("ui.Key");
    pangolin::Var<Eigen::Vector6d>  ui_DeltaPose("ui.Delta");
    pangolin::Var<unsigned int>     ui_nBlur("ui.Blur",1,0,5);
    pangolin::Var<int>              ui_nMaxDisparity("ui.MaxDisp",50,0,100);
    pangolin::Var<float>            ui_fNormC("ui.Norm C",50,0,100);
    pangolin::Var<unsigned int>     ui_nMinU("ui.Min U",30,0,100);
    pangolin::Var<bool>             ui_bDiscHiLo("ui.Discard Hi-Low Pix Values", true, true);
    pangolin::Var<bool>             ui_bBilateralFiltDepth("ui.Bilateral Filter Depth Downsampling", true, true);
    pangolin::Var<int>              ui_nBilateralWinSize("ui.-- Size",5, 1, 20);
    pangolin::Var<float>            ui_gs("ui.-- Spatial",1, 1E-3, 5);
    pangolin::Var<float>            ui_gr("ui.-- Depth Range",0.5, 1E-3, 10);
    pangolin::Var<float>            ui_gc("ui.-- Color Range",10, 1E-3, 20);
    pangolin::Var<float>            ui_fMinPts("ui.Min Points Estimate Threshold",0.30,0,1);
    pangolin::Var<bool>             ui_bUseGlobalMotionModel("ui.Use Global Motion Model", false, true);
    pangolin::Var<float>            ui_fKeyThreshold("ui.Keyframe Threshold",1.0,0,1);
    pangolin::Var<int>              ui_nPyrLevel("ui.Pyramid Level",0,0,MAX_LEVELS-1);
    pangolin::Var<int>              ui_nKeyframeIndex("ui.Keyframe Index ",0,0,400);


    // create a view container
    pangolin::View& guiContainer = pangolin::CreateDisplay()
            .SetBounds(0,1,pangolin::Attach::Pix(300),1)
            .SetLayout(pangolin::LayoutEqual);
    const unsigned int nNumConts = 3;
    for( unsigned int ii = 0; ii <  nNumConts; ii ++ ) {
        pangolin::View& v = pangolin::CreateDisplay();
        v.SetAspect((double)nImgWidth/nImgHeight);
        guiContainer.AddDisplay(v);
    }

    // add 3d view to container
    guiContainer.AddDisplay(glView3D);

    // draw grid on 3D window
    SceneGraph::GLGrid glGrid;
    glGraph.AddChild( &glGrid );

    // gl axis
    SceneGraph::GLAxis glCurPose;
    SceneGraph::GLAxis glPrevPose;
    SceneGraph::GLAxis glKeyPose;
    glGraph.AddChild( &glCurPose );
    glGraph.AddChild( &glPrevPose );
    glGraph.AddChild( &glKeyPose );
    glCurPose.SetScale( 1.0 );
    glPrevPose.SetScale( 0.5 );
    glKeyPose.SetScale( 2.0 );


    // gl path
    GLPath glPath;
//    glGraph.AddChild(&glPath);

    // initialize vbo's
    vector < pangolin::GlBufferCudaPtr* >      vVBO;
    vVBO.resize( MAX_LEVELS );
    vector < pangolin::GlBufferCudaPtr* >      vCBO;
    vCBO.resize( MAX_LEVELS );
    vector < pangolin::GlBufferCudaPtr* >      vIBO;
    vIBO.resize( MAX_LEVELS );
    vector < SceneGraph::GLObject* >           glVBO;
    glVBO.resize( MAX_LEVELS );

    for( int ii = 0; ii < MAX_LEVELS; ii++ ) {
        const unsigned              w = nImgWidth >> ii;
        const unsigned              h = nImgHeight >> ii;

        vVBO[ii] = new pangolin::GlBufferCudaPtr( pangolin::GlArrayBuffer, w, h, GL_FLOAT, 4,
                                       cudaGraphicsMapFlagsWriteDiscard, GL_STREAM_DRAW );
        vCBO[ii] = new pangolin::GlBufferCudaPtr( pangolin::GlArrayBuffer, w, h, GL_UNSIGNED_BYTE, 4,
                                       cudaGraphicsMapFlagsWriteDiscard, GL_STREAM_DRAW );
        vIBO[ii] = new pangolin::GlBufferCudaPtr( pangolin::GlElementArrayBuffer, w, h, GL_UNSIGNED_INT, 2 );

        // add vbo to scenegraph
        glVBO[ii] = new SceneGraph::GLVbo( vVBO[ii], vIBO[ii], vCBO[ii] );
        glKeyPose.AddChild( glVBO[ii] );

    }

    // gpu variables
    Gpu::Pyramid<unsigned char, MAX_LEVELS, Gpu::TargetDevice, Gpu::Manage>     dLeftPyr(nImgWidth, nImgHeight);
    Gpu::Pyramid<unsigned char, MAX_LEVELS, Gpu::TargetDevice, Gpu::Manage>     dRightPyr(nImgWidth, nImgHeight);
    Gpu::Pyramid<unsigned char, MAX_LEVELS, Gpu::TargetDevice, Gpu::Manage>     dKeyPyr(nImgWidth, nImgHeight);
    Gpu::Image< float, Gpu::TargetDevice, Gpu::Manage >                         dKeyDisp( nImgWidth, nImgHeight );
    Gpu::Pyramid< float, MAX_LEVELS, Gpu::TargetDevice, Gpu::Manage >           dKeyDispPyr( nImgWidth, nImgHeight );
    Gpu::Pyramid< float, MAX_LEVELS, Gpu::TargetDevice, Gpu::Manage >           dKeyDispPyrTmp( nImgWidth, nImgHeight );
    Gpu::Pyramid< float, MAX_LEVELS, Gpu::TargetDevice, Gpu::Manage >           dKeyDispPyrNormalized( nImgWidth, nImgHeight );
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage >                 dBlurTmp1( nImgWidth, nImgHeight );
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage >                 dBlurTmp2( nImgWidth, nImgHeight );
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage >                 dWorkspace( nImgWidth*sizeof(Gpu::LeastSquaresSystem<float,6>), nImgHeight );
    Gpu::Image<float4, Gpu::TargetDevice, Gpu::Manage>                          dDebug(nImgWidth, nImgHeight);

    pangolin::ActivateDrawPyramid<unsigned char,MAX_LEVELS>     DrawLeftImg(dLeftPyr,GL_LUMINANCE8,false,true);
    pangolin::ActivateDrawImage<float4>                         DrawDebugImg(dDebug,GL_RGBA32F_ARB,false,true);
    pangolin::ActivateDrawPyramid<float,MAX_LEVELS>             DrawDisparity(dKeyDispPyrNormalized,GL_LUMINANCE32F_ARB,false,true);

    // add images to the container
    guiContainer[0].SetDrawFunction(boost::ref(DrawLeftImg));
    guiContainer[1].SetDrawFunction(boost::ref(DrawDebugImg));
    guiContainer[2].SetDrawFunction(boost::ref(DrawDisparity));

    // Pose variables
    Eigen::Matrix4d T_wk;                                       // keyframe pose in world reference frame
    Eigen::Matrix4d T_wp;                                       // previous pose in world reference frame
    Eigen::Matrix4d T_wc;                                       // current pose in world reference frame
    Eigen::Matrix4d T_kc = Eigen::Matrix4d::Identity();         // current pose relative to keyframe
    Eigen::Matrix4d T_pc = Eigen::Matrix4d::Identity();         // current pose relative to previous pose
    Eigen::Matrix4d T_pc_prev = Eigen::Matrix4d::Identity();    // previous pose update (use if motion model is enabled)

    // Start camera with robot identity
    Eigen::Matrix4d T_vr = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_rv = Eigen::Matrix4d::Identity();
    {
        Eigen::Matrix3d RDFvision;
        RDFvision << 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1;
        Eigen::Matrix3d RDFrobot;
        RDFrobot << 0, 1, 0,
                    0, 0, 1,
                    1, 0, 0;
        T_vr.block < 3, 3 > (0, 0) = RDFvision.transpose( ) * RDFrobot;
        T_rv.block < 3, 3 > (0, 0) = RDFrobot.transpose( ) * RDFvision;
    }
    T_wk = /*T_wr*/ Eigen::Matrix4d::Identity() * T_rv;
    T_wp = /*T_wr*/ Eigen::Matrix4d::Identity() * T_rv;
    T_wc = /*T_wr*/ Eigen::Matrix4d::Identity() * T_rv;
    glPath.PushPose( T_rv );

    // control variables
    bool guiRunning = false;
    bool guiSetKeyframe = false;
    bool guiGo = false;
    bool bNewCapture = false;

    // keyboard callbacks
    pangolin::RegisterKeyPressCallback(' ',[&guiRunning](){ guiRunning = !guiRunning; });
    pangolin::RegisterKeyPressCallback('k',[&guiSetKeyframe](){ guiSetKeyframe = true; });
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + GLUT_KEY_RIGHT,[&guiGo](){ guiGo = true; });
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'r', boost::bind( _HardReset, &T_kc, &T_wk, &T_rv, &guiSetKeyframe, &glPath ) );
    pangolin::RegisterKeyPressCallback('r',[&T_kc](){ T_kc = Eigen::Matrix4d::Identity(); });

#if SAVE_POSES
    // store poses in a file
    ofstream pFile;
    pFile.open("Poses.txt");
#endif


#if USE_PRELOADED
    // pre-load model as a PoseNode Graph
    // eventually get some of these hardcoded values from command line
    vector<PNode> PoseVector;
    {
        // set up filereader
        CameraDevice Cam;
        Cam.SetProperty("DataSourceDir", "./Keyframes" );
        Cam.SetProperty("Channel-0", "Left.*" );
        Cam.SetProperty("Channel-1", "Depth.*" );
        Cam.SetProperty("NumChannels", 2 );

        // init driver
        if( !Cam.InitDriver( "FileReader" ) ) {
                std::cerr << "Invalid input device to load poses." << std::endl;
                exit(0);
        }

        // pose file
        ifstream pFile;
        pFile.open( "Keyframes/Keyframes.txt" );
        if( pFile.is_open() == false ) {
            cerr << "Pose file not found." << endl;
            exit(-1);
        }

        // data containers
        Eigen::Vector6d ReadPose;
        vector< rpg::ImageWrapper > vImgs;

        if( pFile.is_open( ) ) {
            // iterate through pose file
            while( 1 ) {
                // read pose
                pFile >> ReadPose(0) >> ReadPose(1) >> ReadPose(2) >> ReadPose(3) >> ReadPose(4) >> ReadPose(5);

                if( pFile.eof( ) ) {
                    break;
                }

                // read images
                Cam.Capture( vImgs );

                // store data
                PNode tPNode;
                //tPNode.RelPose;
                tPNode.AbsPose = ReadPose;
                tPNode.Image = vImgs[0].Image;
#if DEPTH_MAP
{
    Eigen::Matrix3d             K = CamModel.K(0);
    const float                 fBaseline = CamModel.GetPose()( 1, 3 );
    for( int ii = 0; ii < nImgHeight; ii++ ) {
        for( int jj = 0; jj < nImgWidth; jj++ ) {
            vImgs[1].Image.at<float>(ii,jj) = K(0,0) * fBaseline / vImgs[1].Image.at<float>(ii,jj);
        }
    }
}
#endif

                tPNode.Disparity = vImgs[1].Image;

                PoseVector.push_back(tPNode);
            }
        } else {
            std::cout << "Error opening pose file!" << std::endl;
        }
        pFile.close( );
        std::cout << "Loaded " << PoseVector.size() << " poses from keyframe file." << std::endl;
    }

    // keyframe index
    unsigned int nKeyIdx = 0;

    T_wc = /*T_wr*/ mvl::Cart2T(PoseVector[0].AbsPose) * T_rv;
    T_wk = /*T_wr*/ mvl::Cart2T(PoseVector[0].AbsPose) * T_rv;

#endif

    // set up first keyframe
#if USE_PRELOADED
    dLeftPyr[0].MemcpyFromHost(PoseVector[nKeyIdx].Image.data,nImgWidth);
    Gpu::Image< float, Gpu::TargetHost > hDisp( (float*)PoseVector[nKeyIdx].Disparity.data, nImgWidth, nImgHeight );
#else
    dLeftPyr[0].MemcpyFromHost(vImages[0].Image.data,nImgWidth);
    Gpu::Image< float, Gpu::TargetHost > hDisp( (float*)vImages[2].Image.data, nImgWidth, nImgHeight );
#endif

    // blur & decimate image
    for (int ii = 0; ii < ui_nBlur; ++ii) {
        Gpu::Blur( dLeftPyr[0], dBlurTmp1 );
    }
    Gpu::BlurReduce<unsigned char, MAX_LEVELS, unsigned int>( dLeftPyr, dBlurTmp1, dBlurTmp2 );
    dKeyPyr.CopyFrom(dLeftPyr);

    // downsample depth map
    dKeyDisp.CopyFrom( hDisp );
    dKeyDispPyr[0].CopyFrom( dKeyDisp );
    Gpu::BoxReduce< float, MAX_LEVELS, float >( dKeyDispPyr );

    unsigned int iidx = 0;

    // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Main Loop
    //
    while( !pangolin::ShouldQuit() ) {

        // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Capture
        //
        if( guiRunning || pangolin::Pushed(guiGo)) {
            // capture an image
            pCam->Capture( vImages );
            unsigned int Idx = ui_nImgIdx;
            Idx++;
            ui_nImgIdx = Idx;
            bNewCapture = true;

#if DEPTH_MAP
{
    Eigen::Matrix3d             K = CamModel.K(0);
    const float                 fBaseline = CamModel.GetPose()( 1, 3 );
    for( int ii = 0; ii < nImgHeight; ii++ ) {
        for( int jj = 0; jj < nImgWidth; jj++ ) {
            vImages[2].Image.at<float>(ii,jj) = K(0,0) * fBaseline / vImages[2].Image.at<float>(ii,jj);
        }
    }
}
#endif

        }

        // upload images
        dLeftPyr[0].MemcpyFromHost(vImages[0].Image.data,nImgWidth);
        dRightPyr[0].MemcpyFromHost(vImages[1].Image.data,nImgWidth);

        // blur bottom image
        for (int ii = 0; ii < ui_nBlur; ++ii) {
            Gpu::Blur(dLeftPyr[0], dBlurTmp1);
            Gpu::Blur(dRightPyr[0], dBlurTmp1);
        }

        // reduce and blur rest of pyramid
        Gpu::BlurReduce<unsigned char, MAX_LEVELS, unsigned int>( dLeftPyr, dBlurTmp1, dBlurTmp2 );
        Gpu::BlurReduce<unsigned char, MAX_LEVELS, unsigned int>( dRightPyr, dBlurTmp1, dBlurTmp2 );

        // number of observations
        unsigned int nObs;

        // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Localization
        //
        for( int PyrLvl = MAX_LEVELS-1; PyrLvl >= 0; PyrLvl-- ) {
            for(int ii = 0; ii < g_vPyrCycles[PyrLvl]; ii++ ) {
                const unsigned              w = nImgWidth >> PyrLvl;
                const unsigned              h = nImgHeight >> PyrLvl;

                Eigen::Matrix3d             K = CamModel.K(PyrLvl);
                Sophus::SE3                 sT_pc = Sophus::SE3( T_kc );
                Eigen::Matrix<double,3,4>   KTcp = K * sT_pc.inverse().matrix3x4();
                const float                 fBaseline = (1 << PyrLvl) * CamModel.GetPose()( 1, 3 );

                // build system
                Gpu::LeastSquaresSystem<float,6> LSS = Gpu::PoseRefinementFromDisparityESM(dLeftPyr[PyrLvl],dKeyPyr[PyrLvl],
                                                                                        dKeyDispPyr[PyrLvl],KTcp,ui_fNormC,fBaseline,
                                                                                        K(0,0),K(1,1),K(0,2),K(1,2),
                                                                                        dWorkspace,dDebug.SubImage(w,h),ui_bDiscHiLo);
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
                    Eigen::FullPivLU<Eigen::Matrix<double,3,3> >    lu_JTJ(rLHS);

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
                if( (float)LSS.obs / (float)( w * h ) < ui_fMinPts ) {
                    cerr << "warning(@L" << PyrLvl+1 << "I" << ii+1 << ") LS trashed. " << "Too few pixels!" << endl;
                    continue;
                }

                // everything seems fine... apply update
                T_kc = (T_kc.inverse() * Sophus::SE3::exp(X).matrix()).inverse();

                // only store nObs of last level
                if( PyrLvl == 0 ) {
                    nObs = LSS.obs;
                    ui_fSqErr =  LSS.sqErr / LSS.obs;
                }
            }
        }

        // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // VBO Generation
        //

        // cross-bilateral filter the downsampled disparity maps
        dKeyDispPyr[0].CopyFrom( dKeyDisp );
        Gpu::BoxReduce< float, MAX_LEVELS, float >( dKeyDispPyr );
        if( ui_bBilateralFiltDepth == true ) {
            dKeyDispPyrTmp[0].CopyFrom( dKeyDispPyr[0] );
            for(int ii = 1; ii < MAX_LEVELS; ii++ ) {
                Gpu::BilateralFilter<float,float,unsigned char>(dKeyDispPyrTmp[ii],dKeyDispPyr[ii],dLeftPyr[ii],ui_gs,ui_gr,ui_gc,ui_nBilateralWinSize);
            }
            dKeyDispPyr.CopyFrom(dKeyDispPyrTmp);
        }

        const unsigned              w = nImgWidth >> ui_nPyrLevel;
        const unsigned              h = nImgHeight >> ui_nPyrLevel;

        // Update (VBO) Vertex Buffer Object
        {

            Eigen::Matrix3d                 K = CamModel.K( ui_nPyrLevel );
            const float                     fBaseline = (1 << ui_nPyrLevel) * CamModel.GetPose()( 1, 3 );
            pangolin::CudaScopedMappedPtr   var( *(vVBO[ ui_nPyrLevel]) );
            Gpu::Image< float4 >            dVbo( (float4*)*var, w, h );
            Gpu::DisparityImageToVbo( dVbo, dKeyDispPyr[ ui_nPyrLevel ], fBaseline, K( 0, 0 ), K( 1, 1 ), K( 0, 2 ), K( 1, 2 ) );
        }
        // Generate (IBO) Index Buffer Object for rendering mesh
        {
            pangolin::CudaScopedMappedPtr var( *(vIBO[ ui_nPyrLevel]) );
            Gpu::Image< uint2 >           dIbo( (uint2*)*var, w, h );
            Gpu::GenerateTriangleStripIndexBuffer( dIbo );
        }

        // Update (CBO) Colored Buffered Object for display
        {
            pangolin::CudaScopedMappedPtr var( *(vCBO[ ui_nPyrLevel]) );
            Gpu::Image< uchar4 >          dCbo( (uchar4*)*var, w, h );
            Gpu::ConvertImage< uchar4, unsigned char >( dCbo, dKeyPyr[ ui_nPyrLevel ] );
        }

        // set all VBOs invisible
        for( int ii = 0; ii < MAX_LEVELS; ii ++ ) {
            glVBO[ii]->SetVisible(false);
        }
        glVBO[ ui_nPyrLevel ]->SetVisible(true);


        // update poses
        T_wp = T_wc;
        T_wc = T_wk * T_kc;
        T_pc = mvl::TInv(T_wp) * T_wc;


        // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Keyframe
        //
//        if(pangolin::Pushed(guiSetKeyframe) || (( (float)nObs / (float)( nImgWidth * nImgHeight ) < ui_fKeyThreshold ) && bNewCapture ))
        {

            // convert T_pc update to robotics frame
            Eigen::Vector6d DeltaPose = mvl::T2Cart( T_rv * T_pc * T_vr );

            // check to see if motion model is enabled
            if( ui_bUseGlobalMotionModel ==  true ) {
                if( fabs(DeltaPose(0)) > g_vMotionModel(0)
                        || fabs(DeltaPose(1)) > g_vMotionModel(1)
                        || fabs(DeltaPose(2)) > g_vMotionModel(2)
                        || fabs(DeltaPose(3)) > g_vMotionModel(3)
                        || fabs(DeltaPose(4)) > g_vMotionModel(4)
                        || fabs(DeltaPose(5)) > g_vMotionModel(5)
                  ) {
                    cerr << "warning: Estimate trashed due to motion model. ( " << DeltaPose.transpose() << " )" << endl;
                    T_pc = T_pc_prev;
                    T_wc = T_wp * T_pc;
                    DeltaPose = mvl::T2Cart( T_rv * T_pc * T_vr );
                }
            }


            // update keyframe image
#if USE_PRELOADED
            // otherwise find closest keyframe based on current pose
            Eigen::Vector6d CurPose = mvl::T2Cart( mvl::Cart2T( PoseVector[nKeyIdx].AbsPose ) * T_rv * T_kc * T_vr );
            Eigen::Vector6d PoseError = CurPose - PoseVector[0].AbsPose;
            float DistError = PoseError.norm();
            float BestError = DistError;
            nKeyIdx = 0;
            for( int ii = 1; ii < PoseVector.size(); ii++ ) {
                PoseError = CurPose - PoseVector[ii].AbsPose;
                DistError = PoseError.norm();
                if( DistError < BestError ) {
                    BestError = DistError;
                    nKeyIdx = ii;
                }
            }
            nKeyIdx = ui_nKeyframeIndex;
            ui_nKeyIdx = nKeyIdx;

            dKeyPyr[0].MemcpyFromHost(PoseVector[nKeyIdx].Image.data,nImgWidth);
            // blur & decimate image
            for (int ii = 0; ii < ui_nBlur; ++ii) {
                Gpu::Blur( dKeyPyr[0], dBlurTmp1 );
            }
            Gpu::BlurReduce<unsigned char, MAX_LEVELS, unsigned int>( dKeyPyr, dBlurTmp1, dBlurTmp2 );

            Gpu::Image< float, Gpu::TargetHost > hDisp( (float*)PoseVector[nKeyIdx].Disparity.data, nImgWidth, nImgHeight );

            // update keyframe's pose (in vision frame)
            T_wk = mvl::Cart2T( PoseVector[nKeyIdx].AbsPose ) * T_rv;

            T_kc = mvl::TInv( T_wk ) * T_wc;
#else
            dKeyPyr.CopyFrom(dLeftPyr);

            // update keyframe disparity
            Gpu::Image< float, Gpu::TargetHost > hDisp( (float*)vImages[2].Image.data, nImgWidth, nImgHeight );

            // our current pose is the new keyframe pose
            T_wk = T_wc;

            T_kc = Eigen::Matrix4d::Identity();
#endif

            // update keyframe disparity
            dKeyDisp.CopyFrom( hDisp );


#if SAVE_POSES
            // save poses to a file in ROBOTICS frame
            Eigen::Vector6d Cpc = mvl::T2Cart( T_rv * T_pc * T_vr );
            pFile << Cpc.transpose() << endl;
#endif

            // update GUI
            ui_DeltaPose = DeltaPose;
            T_pc_prev = T_pc;
//            glPath.PushPose ( Eigen::Matrix4d(T_pc) );


        }

        ///------------------------------------------------------------------------------------------------------------

        bNewCapture = false;

        // update and render stuff
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
        glColor4f( 1, 1, 1, 1);

        DrawLeftImg.SetLevel(ui_nPyrLevel);
        dKeyDispPyrNormalized[ui_nPyrLevel].CopyFrom(dKeyDispPyr[ui_nPyrLevel]);
        nppiDivC_32f_C1IR(ui_nMaxDisparity,dKeyDispPyrNormalized[ui_nPyrLevel].ptr,dKeyDispPyrNormalized[ui_nPyrLevel].pitch,dKeyDispPyrNormalized[ui_nPyrLevel].Size());
        DrawDisparity.SetLevel(ui_nPyrLevel);

        glCurPose.SetPose(T_wc);
        glPrevPose.SetPose(T_wp);
        glKeyPose.SetPose(T_wk);
        ui_CurPose = mvl::T2Cart( T_wc * T_vr );
        ui_KeyPose = mvl::T2Cart( T_wk * T_vr );

        pangolin::FinishGlutFrame();

//        sleep(1);
    }

#if SAVE_POSES
    // close poses file
    pFile.close();
#endif

return 0;
}
