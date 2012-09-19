#include <pangolin/pangolin.h>
#include <pangolin/glcuda.h>
#include <kangaroo/kangaroo.h>
#include <sophus/se3.h>
#include <kangaroo/../applications/common/ImageSelect.h>
#include <kangaroo/../applications/common/CameraModelPyramid.h>
#include <SceneGraph/SceneGraph.h>
#include <RPG/Devices.h>
#include <Mvlpp/Mvl.h>
#include <boost/bind.hpp>
#include "Common.h"
#include "ParseCamArgs.h"
#include "StreamHelpers.h"
#include "GLPath.h"
#include "Keyframes.h"

using namespace std;




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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
    // initialize some CVars and global variables
    InitPermutationMatrices();

    // initialize number of iterations to perform at each pyramid level
    // level 0 is finest (ie. biggest image)
    g_vPyrCycles.resize( MAX_PYR_LEVELS );
    g_vPyrCycles << 1, 2, 3, 4, 5;
    g_vPyrFullMask.resize( MAX_PYR_LEVELS );
    g_vPyrFullMask << 1, 1, 1, 1, 0;
    g_vMotionModel << 0.3, 0.2, 0.05, 0.01, 0.01, 0.7;

    // parse camera parameters
    GetPot cl( argc, argv );
    CameraDevice* pCam = ParseCamArgs( &cl );

    // read camera model file
    std::string sSourceDir      = cl.follow( ".", 1, "-sdir"  );
    std::string sCamModFileName = cl.follow( "rcmod.xml", 1, "-rcmod" );
    CameraModelPyramid CamModel( sSourceDir + "/" + sCamModFileName );
    CamModel.PopulatePyramid(MAX_PYR_LEVELS);
    cout << "Reading camera model file..." << endl;
    cout << "-- Image Width: " << CamModel.Width() << endl;
    cout << "-- Image Height: " << CamModel.Height() << endl;
    cout << "-- K Matrix: " << endl;
    cout << CamModel.K() << endl;

    // vector of images captured
    vector< rpg::ImageWrapper > vImages;

    // initial capture for image properties
    pCam->Capture( vImages );

    // image properties
    const unsigned int nImgWidth = vImages[0].Image.cols;
    const unsigned int nImgHeight = vImages[0].Image.rows;
    const unsigned int nThumbHeight = nImgHeight >> MAX_PYR_LEVELS-1;
    const unsigned int nThumbWidth = nImgWidth >> MAX_PYR_LEVELS-1;


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


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
    pangolin::Var<float>            ui_CenterPixelDepth("ui.Center Pixel Depth");
    pangolin::Var<unsigned int>     ui_nBlur("ui.Blur",1,0,5);
    pangolin::Var<float>            ui_fNormC("ui.Norm C",50,0,100);
    pangolin::Var<bool>             ui_bDiscardMaxMin("ui.Discard Max-Min Pix Values", true, true);
    pangolin::Var<bool>             ui_bBilateralFiltDepth("ui.Cross Bilateral Filter (Depth)", true, true);
    pangolin::Var<int>              ui_nBilateralWinSize("ui.-- Size",5, 1, 20);
    pangolin::Var<float>            ui_gs("ui.-- Spatial",1, 1E-3, 5);
    pangolin::Var<float>            ui_gr("ui.-- Depth Range",0.5, 1E-3, 10);
    pangolin::Var<float>            ui_gc("ui.-- Color Range",10, 1E-3, 20);
    pangolin::Var<float>            ui_fMinPts("ui.Min Points Estimate Threshold",0.30,0,1);
    pangolin::Var<bool>             ui_bUseGlobalMotionModel("ui.Use Global Motion Model", false, true);
    pangolin::Var<int>              ui_nPyrLevel("ui.Pyramid Level",0,0,MAX_PYR_LEVELS-1);


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
//    glGraph.AddChild( &glPrevPose );
    glGraph.AddChild( &glKeyPose );


    // gl path
    GLPath glPath;
//    glGraph.AddChild(&glPath);

    // initialize vbo's
    vector < pangolin::GlBufferCudaPtr* >      vVBO;
    vVBO.resize( MAX_PYR_LEVELS );
    vector < pangolin::GlBufferCudaPtr* >      vCBO;
    vCBO.resize( MAX_PYR_LEVELS );
    vector < pangolin::GlBufferCudaPtr* >      vIBO;
    vIBO.resize( MAX_PYR_LEVELS );
    vector < SceneGraph::GLObject* >           glVBO;
    glVBO.resize( MAX_PYR_LEVELS );

    for( int ii = 0; ii < MAX_PYR_LEVELS; ii++ ) {
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
    Gpu::Pyramid<unsigned char, MAX_PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage>     dLeftPyr(nImgWidth, nImgHeight);
    Gpu::Pyramid<unsigned char, MAX_PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage>     dRightPyr(nImgWidth, nImgHeight);
    Gpu::Pyramid<unsigned char, MAX_PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage>     dKeyPyr(nImgWidth, nImgHeight);
    Gpu::Image< float, Gpu::TargetDevice, Gpu::Manage >                             dKeyDepth( nImgWidth, nImgHeight );
    Gpu::Pyramid< float, MAX_PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage >           dKeyDepthPyr( nImgWidth, nImgHeight );
    Gpu::Pyramid< float, MAX_PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage >           dKeyDepthPyrTmp( nImgWidth, nImgHeight );
    Gpu::Pyramid< float, MAX_PYR_LEVELS, Gpu::TargetDevice, Gpu::Manage >           dKeyDepthPyrNormalized( nImgWidth, nImgHeight );
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage >                     dBlurTmp1( nImgWidth, nImgHeight );
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage >                     dBlurTmp2( nImgWidth, nImgHeight );
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage >                     dWorkspace( nImgWidth*sizeof(Gpu::LeastSquaresSystem<float,6>), nImgHeight );
    Gpu::Image<float4, Gpu::TargetDevice, Gpu::Manage>                              dDebug(nImgWidth, nImgHeight);

    pangolin::ActivateDrawPyramid< unsigned char, MAX_PYR_LEVELS >      DrawLeftImg(dLeftPyr,GL_LUMINANCE8,false,true);
    pangolin::ActivateDrawImage< float4 >                               DrawDebugImg(dDebug,GL_RGBA32F_ARB,false,true);
    pangolin::ActivateDrawPyramid< float, MAX_PYR_LEVELS >              DrawDisparity(dKeyDepthPyrNormalized,GL_LUMINANCE32F_ARB,false,true);

    // add images to the container
    guiContainer[0].SetDrawFunction(boost::ref( DrawLeftImg ));
    guiContainer[1].SetDrawFunction(boost::ref( DrawDebugImg ));
    guiContainer[2].SetDrawFunction(boost::ref( DrawDisparity ));


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // Pose variables
    Eigen::Matrix4d T_wk;                                       // keyframe pose in world reference frame
    Eigen::Matrix4d T_wp;                                       // previous pose in world reference frame
    Eigen::Matrix4d T_wc;                                       // current pose in world reference frame
    Eigen::Matrix4d T_kc = Eigen::Matrix4d::Identity();         // current pose relative to keyframe
    Eigen::Matrix4d T_pc = Eigen::Matrix4d::Identity();         // current pose relative to previous pose
    Eigen::Matrix4d T_pc_prev = Eigen::Matrix4d::Identity();    // previous pose update (use if motion model is enabled)

    // Start camera with robot identity
    T_wk = /*T_wr*/ Eigen::Matrix4d::Identity() * g_Trv;
    T_wp = /*T_wr*/ Eigen::Matrix4d::Identity() * g_Trv;
    T_wc = /*T_wr*/ Eigen::Matrix4d::Identity() * g_Trv;
    glPath.PushPose( g_Trv );

    // decimate live image
    dLeftPyr[0].MemcpyFromHost( vImages[0].Image.data, nImgWidth );
    Gpu::BlurReduce< unsigned char, MAX_PYR_LEVELS, unsigned int >( dLeftPyr, dBlurTmp1, dBlurTmp2 );
    cv::Mat ThumbImage( nThumbHeight, nThumbWidth, CV_32FC1 );
    dLeftPyr[MAX_PYR_LEVELS-1].MemcpyToHost( ThumbImage.data );

    // vector of keyframes
    vector< Keyframe_t >        vKeyframes;

    // load keyframes
    LoadKeyframes( &cl, vKeyframes );

    // keyframe index
    cout << "Estimating initial keyframe..." << endl;
    unsigned int    nKeyIdx = FindBestKeyframe( vKeyframes, ThumbImage );
    cout << "... best guess was Keyframe# " << nKeyIdx << "." << endl;

    // initialize pose -- estimated
    T_wk = /*T_wr*/ vKeyframes[nKeyIdx].Pose * g_Trv;
    T_wc = T_wk;

    // set up first keyframe
    dLeftPyr[0].MemcpyFromHost( vKeyframes[nKeyIdx].Image.data, nImgWidth );
    Gpu::Image< float, Gpu::TargetHost > hDepth( (float*)vKeyframes[nKeyIdx].Depth.data, nImgWidth, nImgHeight );

    // blur & decimate image
    for (int ii = 0; ii < ui_nBlur; ++ii) {
        Gpu::Blur( dLeftPyr[0], dBlurTmp1 );
    }
    Gpu::BlurReduce< unsigned char, MAX_PYR_LEVELS, unsigned int >( dLeftPyr, dBlurTmp1, dBlurTmp2 );
    dKeyPyr.CopyFrom(dLeftPyr);

    // downsample depth map
    dKeyDepth.CopyFrom( hDepth );
    dKeyDepthPyr[0].CopyFrom( dKeyDepth );
    Gpu::BoxReduce< float, MAX_PYR_LEVELS, float >( dKeyDepthPyr );


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // control variables
    bool guiRunning = false;
    bool guiSetKeyframe = false;
    bool guiGo = false;
    bool bNewCapture = false;

    // keyboard callbacks
    pangolin::RegisterKeyPressCallback(' ',[&guiRunning](){ guiRunning = !guiRunning; });
    pangolin::RegisterKeyPressCallback('k',[&guiSetKeyframe](){ guiSetKeyframe = true; });
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + GLUT_KEY_RIGHT,[&guiGo](){ guiGo = true; });
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'r', boost::bind( _HardReset, &T_kc, &T_wk, &g_Trv, &guiSetKeyframe, &glPath ) );
    pangolin::RegisterKeyPressCallback('r',[&T_kc](){ T_kc = Eigen::Matrix4d::Identity(); });


    // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Main Loop
    //
    while( !pangolin::ShouldQuit() ) {

        // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Search for Keyframe
        //

        nKeyIdx = FindClosestKeyframe( vKeyframes, T_wc );

        ui_nKeyIdx = nKeyIdx;


        while( true ) {

        // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Load Keyframe
        //

//            _LoadKeyframe( PoseVector, nKeyIdx, dKeyPyr, dKeyDisp, T_wk, T_kc );

        // update keyframe image
        dKeyPyr[0].MemcpyFromHost(vKeyframes[nKeyIdx].Image.data,nImgWidth);
        // blur & decimate image
        for (int ii = 0; ii < ui_nBlur; ++ii) {
            Gpu::Blur( dKeyPyr[0], dBlurTmp1 );
        }
        Gpu::BlurReduce<unsigned char, MAX_PYR_LEVELS, unsigned int>( dKeyPyr, dBlurTmp1, dBlurTmp2 );

        // update keyframe depth map
        Gpu::Image< float, Gpu::TargetHost > hDepth( (float*)vKeyframes[nKeyIdx].Depth.data, nImgWidth, nImgHeight );
        dKeyDepth.CopyFrom( hDepth );

        // update keyframe's pose (in vision frame)
        T_wk = vKeyframes[nKeyIdx].Pose * g_Trv;

        // initialize T_kc from our current global position and that of the keyframe
        T_kc = mvl::TInv( T_wk ) * T_wc;



        // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Generate VBO
        //

        // cross-bilateral filter the downsampled depth maps
        dKeyDepthPyr[0].CopyFrom( dKeyDepth );
        Gpu::BoxReduce< float, MAX_PYR_LEVELS, float >( dKeyDepthPyr );
        if( ui_bBilateralFiltDepth == true ) {
            dKeyDepthPyrTmp[0].CopyFrom( dKeyDepthPyr[0] );
            for(int ii = 1; ii < MAX_PYR_LEVELS; ii++ ) {
                Gpu::BilateralFilter<float,float,unsigned char>(dKeyDepthPyrTmp[ii],dKeyDepthPyr[ii],dLeftPyr[ii],ui_gs,ui_gr,ui_gc,ui_nBilateralWinSize);
            }
            dKeyDepthPyr.CopyFrom(dKeyDepthPyrTmp);
        }

        const unsigned              PyrW = nImgWidth >> ui_nPyrLevel;
        const unsigned              PyrH = nImgHeight >> ui_nPyrLevel;

        // Update (VBO) Vertex Buffer Object
        {

            Eigen::Matrix3d                 K = CamModel.K( ui_nPyrLevel );
            const float                     fBaseline = (1 << ui_nPyrLevel) * CamModel.GetPose()( 1, 3 );
            pangolin::CudaScopedMappedPtr   var( *(vVBO[ ui_nPyrLevel]) );
            Gpu::Image< float4 >            dVbo( (float4*)*var, PyrW, PyrH );
            Gpu::DisparityImageToVbo( dVbo, dKeyDepthPyr[ ui_nPyrLevel ], fBaseline, K( 0, 0 ), K( 1, 1 ), K( 0, 2 ), K( 1, 2 ) );
        }

        // Update (CBO) Colored Buffered Object for display
        {
            pangolin::CudaScopedMappedPtr var( *(vCBO[ ui_nPyrLevel]) );
            Gpu::Image< uchar4 >          dCbo( (uchar4*)*var, PyrW, PyrH );
            Gpu::ConvertImage< uchar4, unsigned char >( dCbo, dKeyPyr[ ui_nPyrLevel ] );
        }

        // set all VBOs invisible
        for( int ii = 0; ii < MAX_PYR_LEVELS; ii ++ ) {
            glVBO[ii]->SetVisible(false);
        }
        glVBO[ ui_nPyrLevel ]->SetVisible(true);



        // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Localization
        //

        unsigned int nObs;                  // number of observations
        for( int PyrLvl = MAX_PYR_LEVELS-1; PyrLvl >= 0; PyrLvl-- ) {
            for(int ii = 0; ii < g_vPyrCycles[PyrLvl]; ii++ ) {
                const unsigned              w = nImgWidth >> PyrLvl;
                const unsigned              h = nImgHeight >> PyrLvl;

                Eigen::Matrix3d             K = CamModel.K(PyrLvl);
                Sophus::SE3                 sT_pc = Sophus::SE3( T_kc );
                Eigen::Matrix<double,3,4>   KTcp = K * sT_pc.inverse().matrix3x4();
                const float                 fBaseline = (1 << PyrLvl) * CamModel.GetPose()( 1, 3 );

                // build system
                Gpu::LeastSquaresSystem<float,6> LSS = Gpu::PoseRefinementFromDepthESM(dLeftPyr[PyrLvl],dKeyPyr[PyrLvl],
                                                                                        dKeyDepthPyr[PyrLvl],KTcp,ui_fNormC,fBaseline,
                                                                                        K(0,0),K(1,1),K(0,2),K(1,2),
                                                                                        dWorkspace,dDebug.SubImage(w,h),ui_bDiscardMaxMin);

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

        // update pose
        T_wc = T_wk * T_kc;

        unsigned int nNewKeyframe = FindClosestKeyframe( vKeyframes, T_wc );

        if( nNewKeyframe == nKeyIdx ) {
            break;
        }

        nKeyIdx = nNewKeyframe;

        }


        // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Capture
        //
        if( guiRunning || pangolin::Pushed(guiGo) ) {

            /*            MOTION MODEL STUFF
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
            */

            // accept estimate
            T_pc = mvl::TInv(T_wp) * T_wc;
            T_wp = T_wc;

            // capture an image
            pCam->Capture( vImages );
            unsigned int Idx = ui_nImgIdx;
            Idx++;
            ui_nImgIdx = Idx;
            bNewCapture = true;
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
        Gpu::BlurReduce<unsigned char, MAX_PYR_LEVELS, unsigned int>( dLeftPyr, dBlurTmp1, dBlurTmp2 );
        Gpu::BlurReduce<unsigned char, MAX_PYR_LEVELS, unsigned int>( dRightPyr, dBlurTmp1, dBlurTmp2 );



        ///------------------------------------------------------------------------------------------------------------


        // update and render stuff
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
        glColor4f( 1, 1, 1, 1);

        // display center pixel's depth info
        ui_CenterPixelDepth = vKeyframes[nKeyIdx].Depth.at<float>( nImgHeight/2, nImgWidth/2 );

        // draw left image
        DrawLeftImg.SetLevel(ui_nPyrLevel);

        // normalize and draw disparity
        dKeyDepthPyrNormalized[ui_nPyrLevel].CopyFrom(dKeyDepthPyr[ui_nPyrLevel]);
        float fMaxDepth = *max_element( vKeyframes[nKeyIdx].Depth.begin<float>(), vKeyframes[nKeyIdx].Depth.end<float>() );
        nppiDivC_32f_C1IR( fMaxDepth, dKeyDepthPyrNormalized[ui_nPyrLevel].ptr, dKeyDepthPyrNormalized[ui_nPyrLevel].pitch,
                           dKeyDepthPyrNormalized[ui_nPyrLevel].Size() );
        DrawDisparity.SetLevel(ui_nPyrLevel);

        // update poses
        glCurPose.SetPose(T_wc);
        glPrevPose.SetPose(T_wp);
        glKeyPose.SetPose(T_wk);
        ui_CurPose = mvl::T2Cart( T_wc * g_Tvr );
        ui_KeyPose = mvl::T2Cart( T_wk * g_Tvr );
        ui_DeltaPose = mvl::T2Cart( g_Trv * T_kc * g_Tvr );

        pangolin::FinishGlutFrame();

    }


return 0;
}
