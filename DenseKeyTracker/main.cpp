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
#include <CVars/CVar.h>
#include "ParseArgs.h"
#include "CVarHelpers.h"
#include "GLPath.h"

using namespace std;

struct Keyframe_t {
    Eigen::Vector6d RelPose;
    Eigen::Vector6d AbsPose;
    cv::Mat         Image;
    cv::Mat         Disparity;
};

const int g_nMaxLevels = 5;

/// CVars
Eigen::Matrix<int,1,Eigen::Dynamic>& g_vPyrCycles = CVarUtils::CreateCVar( "Tracker.PyrCycles", Eigen::Matrix<int,1,Eigen::Dynamic>(), "Number of cycles per pyramid level." );
Eigen::Matrix<int,1,Eigen::Dynamic>& g_vPyrFullMask = CVarUtils::CreateCVar( "Tracker.PyrFullMask", Eigen::Matrix<int,1,Eigen::Dynamic>(), "Set 1 for full estimate, 0 for rotation only estimates." );
unsigned int& g_nPoseDisplay = CVarUtils::CreateCVar("Gui.PoseDisplay", 5u, "Number of axis to draw for poses." );

std::ostream& operator<< (std::ostream& os, const Eigen::Vector6d& v)
{
    os << "( " << fixed << setprecision(1) << showpos << v(0) << ", " << v(1) << ", " << v(2) << ", " << v(3) << ", " << v(4) << ", " << v(5) << " )";
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


void _HardReset( Eigen::Matrix4d* T_pc,
             Eigen::Matrix4d* T_wp,
             Eigen::Matrix4d* T_rv,
             bool* guiSetPrevious,
             GLPath* glPath,
             bool* bVboInit
               )

{
    *T_pc = Eigen::Matrix4d::Identity();
    *T_wp = Eigen::Matrix4d( *T_rv );
    *guiSetPrevious = true;
    glPath->InitReset ();
    glPath->PushPose( *T_rv );
    *bVboInit = false;
}

int main(int argc, char** argv)
{    
    // initialize number of iterations to perform at each pyramid level
    // level 0 is finest (ie. biggest image)
    g_vPyrCycles.resize( g_nMaxLevels );
    g_vPyrCycles << 1, 2, 3, 4, 8;
    g_vPyrFullMask.resize( g_nMaxLevels );
    g_vPyrFullMask << 1, 1, 1, 1, 0;

    // parse parameters
    CameraDevice* pCam = ParseArgs( argc, argv );

    // read camera model file
    std::string sCamModFileName = pCam->GetProperty( "CamModelFile", "rcmod.xml" );
    CameraModelPyramid CamModel( sCamModFileName );
    CamModel.PopulatePyramid(g_nMaxLevels);

    // vector of images captured
    vector< rpg::ImageWrapper > vImages;

    // initial capture for image properties
    pCam->Capture( vImages );

    const unsigned int nImgWidth = vImages[1].Image.cols;
    const unsigned int nImgHeight = vImages[0].Image.rows;

    // create a GUI window
    pangolin::CreateGlutWindowAndBind("Dense Tracker",1680,1050);
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
    pangolin::Var<unsigned int> nImgIdx("ui.Image ID", 0);
    pangolin::Var<Eigen::Vector6d> uiCurPose("ui.Cur");
    pangolin::Var<unsigned int> nKeyIdx("ui.Key ID");
    pangolin::Var<Eigen::Vector6d> uiKeyPose("ui.Key");
    pangolin::Var<Eigen::Vector6d> uiDeltaPose("ui.Delta");
    pangolin::Var<unsigned int> nBlur("ui.Blur",1,0,5);
    pangolin::Var<int> nMaxDisparity("ui.MaxDisp",16,0,100);
    pangolin::Var<int> nMedianFilter("ui.MedianFilter",50,0,100);
    pangolin::Var<float> fNormC("ui.Norm C",50,0,100);
    pangolin::Var<float> fSqErr("ui.Mean Sq Error");
    pangolin::Var<float> fKeyThreshold("ui.Keyframe Threshold",0.75,0,1);
    pangolin::Var<int> nPyramid("ui.Pyramid",0,0,g_nMaxLevels-1);

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
    glGraph.AddChild( &glCurPose );
    glGraph.AddChild( &glPrevPose );

    // gl path
    GLPath glPath;
    glGraph.AddChild(&glPath);

    // initialize vbo's
    pangolin::GlBufferCudaPtr vbo( pangolin::GlArrayBuffer, nImgWidth, nImgHeight, GL_FLOAT, 4,
                                   cudaGraphicsMapFlagsWriteDiscard, GL_STREAM_DRAW );
    pangolin::GlBufferCudaPtr cbo( pangolin::GlArrayBuffer, nImgWidth, nImgHeight, GL_UNSIGNED_BYTE, 4,
                                   cudaGraphicsMapFlagsWriteDiscard, GL_STREAM_DRAW );
    pangolin::GlBufferCudaPtr ibo( pangolin::GlElementArrayBuffer, nImgWidth, nImgHeight, GL_UNSIGNED_INT, 2 );

    // create vbo
    SceneGraph::GLVbo glVBO( &vbo, &ibo, &cbo );

    // Generate Index Buffer Object for rendering mesh
    {
        pangolin::CudaScopedMappedPtr var( ibo );
        Gpu::Image< uint2 >        dIbo( (uint2*)*var, nImgWidth, nImgHeight );
        Gpu::GenerateTriangleStripIndexBuffer( dIbo );
    }
    glPrevPose.AddChild(&glVBO);
    bool bVboInit = false;

    // gpu variables
    Gpu::Pyramid<unsigned char, g_nMaxLevels, Gpu::TargetDevice, Gpu::Manage>   dLeftPyr(nImgWidth, nImgHeight);
    Gpu::Pyramid<unsigned char, g_nMaxLevels, Gpu::TargetDevice, Gpu::Manage>   dRightPyr(nImgWidth, nImgHeight);
    Gpu::Pyramid<unsigned char, g_nMaxLevels, Gpu::TargetDevice, Gpu::Manage>   dKeyPyr(nImgWidth, nImgHeight);
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage >                 dDispInt( nImgWidth, nImgHeight );
    Gpu::Pyramid< float, g_nMaxLevels, Gpu::TargetDevice, Gpu::Manage >         dDispPyr( nImgWidth, nImgHeight );
    Gpu::Pyramid< float, g_nMaxLevels, Gpu::TargetDevice, Gpu::Manage >         dDispPyrC( nImgWidth, nImgHeight );
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage >                 dBlurTmp1( nImgWidth, nImgHeight );
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage >                 dBlurTmp2( nImgWidth, nImgHeight );
    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage >                 dWorkspace( nImgWidth*sizeof(Gpu::LeastSquaresSystem<float,6>), nImgHeight );
    Gpu::Image<float4, Gpu::TargetDevice, Gpu::Manage>                          dDebug(nImgWidth, nImgHeight);

    pangolin::ActivateDrawPyramid<unsigned char,g_nMaxLevels> DrawLeftImg(dLeftPyr,GL_LUMINANCE8,false,true);
    pangolin::ActivateDrawImage<float4> DrawDebugImg(dDebug,GL_RGBA32F_ARB,false,true);
    pangolin::ActivateDrawPyramid<float,g_nMaxLevels> DrawDisparity(dDispPyrC,GL_LUMINANCE32F_ARB,false,true);

    // add images to the container
    guiContainer[0].SetDrawFunction(boost::ref(DrawLeftImg));
    guiContainer[1].SetDrawFunction(boost::ref(DrawDebugImg));
    guiContainer[2].SetDrawFunction(boost::ref(DrawDisparity));

    // Pose variables
    Eigen::Matrix4d T_wp;
    Eigen::Matrix4d T_pc = Eigen::Matrix4d::Identity();

    // Start camera with robot identity
    Eigen::Matrix3d RDFvision;
    RDFvision << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::Matrix3d RDFrobot;
    RDFrobot << 0, 1, 0, 0, 0, 1, 1, 0, 0;
    Eigen::Matrix4d T_vr = Eigen::Matrix4d::Identity( );
    Eigen::Matrix4d T_rv = Eigen::Matrix4d::Identity( );
    T_vr.block < 3, 3 > (0, 0) = RDFvision.transpose( ) * RDFrobot;
    T_rv.block < 3, 3 > (0, 0) = RDFrobot.transpose( ) * RDFvision;
    T_wp = /*T_wr*/ Eigen::Matrix4d::Identity() * T_rv;
    glPath.PushPose( T_rv );

    // gui control variables
    bool guiRunning = false;
    bool guiSetKeyframe = true;
    bool guiGo = false;

    // keyboard callbacks
    pangolin::RegisterKeyPressCallback(' ',[&guiRunning](){ guiRunning = !guiRunning; });
    pangolin::RegisterKeyPressCallback('k',[&guiSetKeyframe](){ guiSetKeyframe = true; });
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + GLUT_KEY_RIGHT,[&guiGo](){ guiGo = true; });
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'r', boost::bind( _HardReset, &T_pc, &T_wp, &T_rv, &guiSetKeyframe, &glPath, &bVboInit ) );
    pangolin::RegisterKeyPressCallback('r',[&T_pc](){ T_pc = Eigen::Matrix4d::Identity(); });

    // pre-load model as a PoseNode Graph
    // eventually get some of these hardcoded values from command line
    vector<Keyframe_t> PoseVector;
    {
        // set up filereader
        CameraDevice Cam;
        Cam.SetProperty("DataSourceDir", "./Keyframes" );
        Cam.SetProperty("Channel-0", "Left.*" );
        Cam.SetProperty("Channel-1", "depth.*" );
        Cam.SetProperty("NumChannels", 2 );

        // init driver
        if( !Cam.InitDriver( "FileReader" ) ) {
                std::cerr << "Invalid input device to load poses." << std::endl;
                exit(0);
        }

        // pose file
        ifstream pFile;
        pFile.open( "Keyframes/poses.txt" );

        // data containers
        Eigen::Vector6d RelPose;
        Eigen::Matrix4d AbsPose = Eigen::Matrix4d::Identity();
        vector< rpg::ImageWrapper > vImgs;

        if( pFile.is_open( ) ) {
            // iterate through pose file
            while( 1 ) {
                // read pose
                pFile >> RelPose(0) >> RelPose(1) >> RelPose(2) >> RelPose(3) >> RelPose(4) >> RelPose(5);

                if( pFile.eof( ) ) {
                    break;
                }

                AbsPose = AbsPose * mvl::Cart2T( RelPose );

                // read images
                Cam.Capture( vImgs );

                // store data
                Keyframe_t tPNode;
                tPNode.RelPose = RelPose;
                tPNode.AbsPose = mvl::T2Cart( AbsPose );
                tPNode.Image = vImgs[0].Image;
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
    unsigned int KeyIdx;

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
            unsigned int Idx = nImgIdx;
            Idx++;
            nImgIdx = Idx;
        }

        // upload images
        dLeftPyr[0].MemcpyFromHost(vImages[0].Image.data,nImgWidth);
        dRightPyr[0].MemcpyFromHost(vImages[1].Image.data,nImgWidth);

        // blur bottom image
        for (int ii = 0; ii < nBlur; ++ii) {
            Gpu::Blur(dLeftPyr[0], dBlurTmp1);
            Gpu::Blur(dRightPyr[0], dBlurTmp1);
        }

        // reduce and blur rest of pyramid
        Gpu::BlurReduce<unsigned char, g_nMaxLevels, unsigned int>( dLeftPyr, dBlurTmp1, dBlurTmp2 );
        Gpu::BlurReduce<unsigned char, g_nMaxLevels, unsigned int>( dRightPyr, dBlurTmp1, dBlurTmp2 );

        // number of observations
        unsigned int nObs;

        // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Localization
        //
        if( bVboInit ) {
            for( int PyrLvl = g_nMaxLevels-1; PyrLvl >= 0; PyrLvl-- ) {
                for(int ii = 0; ii < g_vPyrCycles[PyrLvl]; ii++ ) {
                    const unsigned w = nImgWidth >> PyrLvl;
                    const unsigned h = nImgHeight >> PyrLvl;

                    Eigen::Matrix3d K = CamModel.K(PyrLvl);
                    Sophus::SE3 sT_pc = Sophus::SE3( T_pc );
                    Eigen::Matrix<double,3,4> KTcp = K * sT_pc.inverse().matrix3x4();
                    const float fBaseline = (1 << PyrLvl) * CamModel.GetPose()( 1, 3 );

                    // build system
                    Gpu::LeastSquaresSystem<float,6> LSS = Gpu::PoseRefinementFromDisparityESM(dLeftPyr[PyrLvl],dKeyPyr[PyrLvl],
                                                                                            dDispPyr[PyrLvl],KTcp,fNormC,fBaseline,
                                                                                            K(0,0),K(1,1),K(0,2),K(1,2),
                                                                                            dWorkspace,dDebug.SubImage(w,h),10,false);
                    Eigen::Matrix<double,6,6> LHS = LSS.JTJ;
                    Eigen::Vector6d RHS = LSS.JTy;

                    // solve system
                    Eigen::Vector6d X;

                    // check if we are solving only for rotation, or full estimate
                    if( g_vPyrFullMask(PyrLvl) != 0 ) {
                        Eigen::FullPivLU<Eigen::Matrix<double,6,6> > lu_JTJ(LHS);

                        // check degenerate system
                        if( lu_JTJ.rank() < 6 ) {
                            cerr << "warning(@L" << PyrLvl << "I" << ii << ") LS trashed. " << "Rank: " << lu_JTJ.rank() << endl;
                            continue;
                        }

                        X = - (lu_JTJ.solve(RHS));
                    } else {
                        // extract rotation information only
                        Eigen::Matrix<double,3,3> rLHS = LHS.block<3,3>(3,3);
                        Eigen::Vector3d rRHS = RHS.tail(3);
                        Eigen::FullPivLU<Eigen::Matrix<double,3,3> > lu_JTJ(rLHS);

                        // check degenerate system
                        if( lu_JTJ.rank() < 3 ) {
                            cerr << "warning(@L" << PyrLvl << "I" << ii << ") LS trashed. " << "Rank: " << lu_JTJ.rank() << endl;
                            continue;
                        }

                        Eigen::Vector3d rX;
                        rX = - (lu_JTJ.solve(rRHS));

                        // pack solution
                        X.setZero();
                        X.tail(3) = rX;
                    }

                    // if we have too few observations, discard estimate
                    if( (float)LSS.obs / (float)( w * h ) < 0.20 ) {
                        cerr << "warning(@L" << PyrLvl << "I" << ii << ") LS trashed. " << "Too few pixels!" << endl;
                        continue;
                    }

                    // everything seems fine... apply update
                    T_pc = (T_pc.inverse() * Sophus::SE3::exp(X).matrix()).inverse();
                    fSqErr =  LSS.sqErr / LSS.obs;

                    // only store nObs of last level
                    if( PyrLvl == 0 ) {
                        nObs = LSS.obs;
                    }
                }
            }
        }

        // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Keyframe & VBO Generation
        //
        if( pangolin::Pushed(guiSetKeyframe) || ( (float)nObs / (float)( nImgWidth * nImgHeight ) < fKeyThreshold ) )
        {
            // if this is the first time we come in here, load first keyframe
            if( bVboInit == false ) {
                KeyIdx = 0;
            } else {
                // Update 'previous' position to current position
                cout << "Previous Pose: " << mvl::T2Cart( T_wp * T_vr ).transpose() << endl;
                glPath.PushPose ( Eigen::Matrix4d(T_pc) );
    //            T_wp = T_wp * T_pc;
                cout << "Tpc: " << mvl::T2Cart(T_rv * T_pc * T_vr).transpose() << endl;
                cout << "Keyframe Pose: " << PoseVector[KeyIdx].AbsPose.transpose() << endl;
                T_wp = mvl::Cart2T( PoseVector[KeyIdx].AbsPose ) * T_rv * T_pc;
                T_pc = Eigen::Matrix4d::Identity();

                // otherwise find closest keyframe based on current pose
                Eigen::Vector6d CurPose = mvl::T2Cart( T_wp * T_vr );
                Eigen::Vector6d PoseError = CurPose - PoseVector[0].AbsPose;
                float DistError = PoseError.norm();
                float BestError = DistError;
                KeyIdx = 0;
                for( int ii = 1; ii < PoseVector.size(); ii++ ) {
                    PoseError = CurPose - PoseVector[ii].AbsPose;
                    DistError = PoseError.norm();
                    if( DistError < BestError ) {
                        BestError = DistError;
                        KeyIdx = ii;
                    }
                }
                cout << "Current Pose: " << CurPose.transpose() << endl;
                cout << "Best Pose: " << PoseVector[KeyIdx].AbsPose.transpose() << endl;
                cout << "Keyframe Index: " << KeyIdx << endl;
            }


            // load image
            dKeyPyr[0].MemcpyFromHost( PoseVector[KeyIdx].Image.data, nImgWidth);

            // blur bottom image
            /*
            for (int ii = 0; ii < nBlur; ++ii) {
                Gpu::Blur(dKeyPyr[0], dBlurTmp1);
            }
            */

            // reduce and blur rest of pyramid
            Gpu::BlurReduce<unsigned char, g_nMaxLevels, unsigned int>( dKeyPyr, dBlurTmp1, dBlurTmp2 );

            // load disparity
            Gpu::Image< float, Gpu::TargetHost > hDisp( (float*)PoseVector[KeyIdx].Disparity.data, nImgWidth, nImgHeight );
            dDispPyr[0].CopyFrom( hDisp );
            Gpu::FilterDispGrad(dDispPyr[0], dDispPyr[0], 2.0);
            Gpu::BoxReduce< float, g_nMaxLevels, float >( dDispPyr );

            // Update CBO
            {
                pangolin::CudaScopedMappedPtr var( cbo );
                Gpu::Image< uchar4 >       dCbo( (uchar4*)*var, nImgWidth, nImgHeight );
                Gpu::ConvertImage< uchar4, unsigned char >( dCbo, dKeyPyr[0] );
            }

            // Update VBO
            {
                Eigen::Matrix3d K = CamModel.K(0);
                const float fBaseline = CamModel.GetPose()( 1, 3 );
                pangolin::CudaScopedMappedPtr var( vbo );
                Gpu::Image< float4 >       dVbo( (float4*)*var, nImgWidth, nImgHeight );
                Gpu::DisparityImageToVbo( dVbo, dDispPyr[0], fBaseline, K( 0, 0 ), K( 1, 1 ), K( 0, 2 ), K( 1, 2 ) );
            }


            bVboInit = true;
        }

        ///------------------------------------------------------------------------------------------------------------

        // update and render stuff
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
        glColor4f( 1, 1, 1, 1);

        DrawLeftImg.SetLevel(nPyramid);
        dDispPyrC[nPyramid].CopyFrom(dDispPyr[nPyramid]);
        nppiDivC_32f_C1IR(nMaxDisparity,dDispPyrC[nPyramid].ptr,dDispPyrC[nPyramid].pitch,dDispPyrC[nPyramid].Size());
        DrawDisparity.SetLevel(nPyramid);
        glCurPose.SetPose( Eigen::Matrix4d(T_wp * T_pc) );
        glPrevPose.SetPose(T_wp);
        uiCurPose = mvl::T2Cart(T_wp * T_pc * T_vr );
        uiDeltaPose = mvl::T2Cart(T_rv * T_pc * T_vr );
        nKeyIdx = KeyIdx;
        uiKeyPose = PoseVector[KeyIdx].AbsPose;

        pangolin::FinishGlutFrame();
    }

return 0;
}
