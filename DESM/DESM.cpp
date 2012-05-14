#include <SimpleGui/Gui.h>

//#include <CVarHelpers.h>

#include <Mvlpp/SE3.h>

#include "GLHeightMap.h"
#include "PeaksHeightMap.h"


// Global Vars
GLImage     glImgDiff;
int         g_nImgWidth = 128;
int         g_nImgHeight = 128;
bool        g_DoESM = false;

// Global CVars
bool& g_bShowFrustum = CVarUtils::CreateCVar( "ShowFrustum", true, "Show camera viewing frustum." );

// Camera
GLSimCam Cam;
Eigen::Matrix4d g_dPose = GLCart2T( -40, 0, -5, 0, 0, 0 ); // initial camera pose
//Eigen::Matrix4d g_dPose = GLCart2T( 0, 0, -40, 0, -M_PI / 2, 0 ); // initial camera pose
float g_fTurnrate = 0;
float g_fSpeed = 0;

// Reference Camera -- this is only used to show on screen estimated camera pose
GLSimCam EstCam;


/////////////////////////////////////////////////////////////////////////////////////////
///
class GuiWindow : public GLWindow
{
public:

    GuiWindow( int x, int y, int w, int h, const char *l = 0 ) : GLWindow( x, y, w, h, l )
    {
    }

    virtual int handle( int e )
    {
        if( e == FL_KEYBOARD && !m_Console.IsOpen( ) ) {
            switch( Fl::event_key( ) ) {
                case 'a': case 'A':
                    g_fTurnrate -= 0.01;
                    break;
                case 's': case 'S':
                    g_fSpeed -= 0.2;
                    break;
                case 'd': case 'D':
                    g_fTurnrate += 0.01;
                    break;
                case 'w': case 'W':
                    g_fSpeed += 0.2;
                    break;
                case ' ':
                    g_fSpeed = 0;
                    g_fTurnrate = 0;
                    break;
                case 'e':
                    g_DoESM = true;
                    break;
            }
        }
        return SimpleDefaultEventHandler( e );
    }
};

/////////////////////////////////////////////////////////////////////////////////////////
void UpdateCameraPose( GLWindow*, void* )
{
    g_dPose = g_dPose * GLCart2T( g_fSpeed, 0, 0, 0, 0, g_fTurnrate );
    Cam.SetPose( g_dPose );

    glEnable( GL_LIGHTING );
    glEnable( GL_LIGHT0 );
    glClearColor( 0.0, 0.0, 0.0, 1 );

    Cam.RenderToTexture( ); // will render to texture, then copy texture to CPU memory
}

/////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector3d _BackProject( int X, int Y, double Depth ) {
    // get camera intrinsics
    Eigen::Matrix3d K = Cam.GetKMatrix();
    double cx = K(0,2);
    double cy = K(1,2);
    double fx = K(0,0);
    double fy = K(1,1);

    Eigen::Vector3d P;
    P(0) = Depth * ((X - cx) / fx);
    P(1) = Depth * ((Y - cy) / fy);
    P(2) = Depth;

    return P;
}

/////////////////////////////////////////////////////////////////////////////////////////
void _WarpDepthMap( Eigen::VectorXf& vDM, Eigen::Vector6d Trf ) {
    // get rotation matrix and translation vector
    Eigen::Matrix3d R = mvl::Cart2R( Trf.block<3,1>(0,0) );
    Eigen::Vector3d T = Trf.block<3,1>(3,0);


    Eigen::Vector3d P;
    for( int ii = 0; ii < g_nImgHeight; ii++ ) {
        for( int jj = 0; jj < g_nImgWidth; jj++ ) {
            P = _BackProject( jj, ii, vDM[ii*g_nImgWidth + jj] );
            P = R * P;
            P = P + T;
            vDM[ii*g_nImgWidth + jj] = P(2);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void EstimateCameraPose( GLWindow*, void* )
{
    if( g_DoESM ) {
    Eigen::VectorXf vDepth;
    Eigen::VectorXf vEstDepth;

    // resize vectors
    vDepth.resize( g_nImgWidth * g_nImgHeight );
    vEstDepth.resize( g_nImgWidth * g_nImgHeight );

    // populate vectors
    // assuming depth is not normalized
    Cam.CaptureDepth( vDepth.data() );
    EstCam.CaptureDepth( vEstDepth.data() );

    /* */
    // sanity check... this shows the depth cam is fucked!
    std::cout << "Size: " << vDepth.size( ) << std::endl;
    for( int ii = 0; ii < g_nImgHeight; ii++ ) {
        for( int jj = 0; jj < g_nImgWidth; jj++ ) {
         printf( "%.2f ", vDepth[ii*g_nImgWidth+jj] );
        }
    }
    /**/

    // estimated pose
    Eigen::Vector6d dPose = Eigen::Vector6d::Zero();

    Eigen::VectorXf error;
    error = vEstDepth - vDepth;

    while( error.norm() > 1.0 ) {

        // LHS = Hessian = Jt * J
        Eigen::Matrix<double,6,6> LHS = Eigen::Matrix<double,6,6>::Zero();

        // RHS = Jt * e
        Eigen::Vector6d RHS = Eigen::Vector6d::Zero();

        for( int ii = 0; ii < g_nImgHeight; ii++ ) {
            for( int jj = 0; jj < g_nImgWidth; jj++ ) {
                Eigen::Vector3d P, Pe;
                P = _BackProject( jj, ii, vDepth[ii*g_nImgWidth + jj] );
                Pe = _BackProject( jj, ii, vEstDepth[ii*g_nImgWidth + jj] );

                // build Jacobian
                Eigen::Matrix3d R, Pr;
                Pr << P(0), 0,    0,
                      0,    P(1), 0,
                      0,    0,    P(2);
                R = mvl::Cart2R( dPose.block<3,1>(0,0) );
                R = R * Pr;
                Eigen::Matrix<double,3,6> J;
                J << 0, 0, 0, 1, 0, 0,
                     0, 0, 0, 0, 1, 0,
                     0, 0, 0, 0, 0, 1;
                J.block<3,3>(0,0) = R;

                // estimate LHS (Hessian)
                LHS += J.transpose() * J;

                // estimate RHS (error)
                RHS += J.transpose() * (Pe - P);
            }
        }

        // calculate deltaPose as Hinv * Jt * error
        Eigen::Vector6d deltaPose;
//        deltaPose = LHS.inverse() * RHS;
        deltaPose = LHS.ldlt().solve(RHS);


        // update dPose += deltaPose
        dPose += deltaPose;

        // warp
        _WarpDepthMap( vEstDepth, deltaPose );

        // move camera to new pose and re-render
        EstCam.SetPose( mvl::Cart2T(dPose) );
        EstCam.RenderToTexture( );

        // calculate new error
        error = vEstDepth - vDepth;
        std::cout << "Error is: " << error.norm() << std::endl;
    }

    std::cout << "Estimated Pose is: " << dPose.transpose() << std::endl;
    g_DoESM = false;

    }
}


/////////////////////////////////////////////////////////////////////////////////////////
void ShowCameraAndTextures( GLWindow*, void* )
{
    if( g_bShowFrustum ) {
        // show the camera
        Cam.DrawCamera( );
        EstCam.DrawCamera( );
    }

    /// show textures
    if( Cam.HasRGB( ) ) {
        DrawTextureAsWindowPercentage( Cam.RGBTexture( ), Cam.ImageWidth( ),
                Cam.ImageHeight( ), 0, 0.66, 0.33, 1 );
        DrawBorderAsWindowPercentage( 0, 0.66, 0.33, 1 );
    }

    if( EstCam.HasRGB( ) ) {
        DrawTextureAsWindowPercentage( EstCam.RGBTexture( ), EstCam.ImageWidth( ),
                EstCam.ImageHeight( ), 0.33, 0.66, 0.66, 1 );
        DrawBorderAsWindowPercentage( 0.33, 0.66, 0.66, 1 );
    }

    // draw difference image
    Eigen::Matrix<unsigned char, Eigen::Dynamic, 1> vRGB;
    Eigen::Matrix<unsigned char, Eigen::Dynamic, 1> vEstRGB;

    // resize vectors
    vRGB.resize( g_nImgWidth * g_nImgHeight * 3 );
    vEstRGB.resize( g_nImgWidth * g_nImgHeight * 3 );

    // populate vectors
    Cam.CaptureRGB( vRGB.data() );
    EstCam.CaptureRGB( vEstRGB.data() );

    // calculate difference
    vEstRGB = vEstRGB - vRGB;

    //glImgDiff.SetImage( vEstRGB.data(), g_nImgWidth, g_nImgHeight, GL_RGB, GL_UNSIGNED_BYTE );
    //glImgDiff.SetSizeAsPercentageOfWindow( 0.66, 0.66, 1, 1);
    //DrawBorderAsWindowPercentage( 0.66, 0.66, 1, 1 );
}


/////////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    // register CVars
//    CVarUtils::AttachCVar( "CamPose", &g_dPose );

    // parse arguments
    GetPot cl( argc, argv );
    std::string sMesh = cl.follow( "Terrain.ac", 1, "-mesh" );

    // init window
    GuiWindow* pWin = new GuiWindow( 0, 0, 1024, 768, "Dense ESM" );

    // load mesh
    const struct aiScene* pScene;
    pScene = aiImportFile( sMesh.c_str( ), aiProcess_Triangulate | aiProcess_GenNormals );

    GLGrid glGrid;
    glGrid.SetPerceptable( false );

    GLMesh glMesh;
    glMesh.Init( pScene );

    PeaksHeightMap glPHM;
    GLHeightMap glHM( &glPHM );

    glImgDiff.InitReset();
    glImgDiff.SetPerceptable( false );

    // register objects
    pWin->AddChildToRoot( &glHM );
    pWin->AddChildToRoot( &glGrid );
    //pWin->AddChildToRoot( &glImgDiff );

    Eigen::Matrix3d dK; // computer vision K matrix
    dK <<   g_nImgWidth,    0,              g_nImgWidth / 2,
            0,              g_nImgHeight,   g_nImgHeight / 2,
            0,              0,              1;

    Cam.Init( &pWin->SceneGraph( ), g_dPose, dK, g_nImgWidth, g_nImgHeight, eSimCamDepth | eSimCamRGB );
    EstCam.Init( &pWin->SceneGraph( ), g_dPose, dK, g_nImgWidth, g_nImgHeight, eSimCamDepth | eSimCamRGB );

    glEnable( GL_LIGHT0 ); // activate light0
    glEnable( GL_LIGHTING ); // enable lighting

    pWin->LookAt( -70, -70, -50, 0, 0, 0, 0, 0, -1 );


    // add our callbacks
    pWin->AddPreRenderCallback( UpdateCameraPose, NULL );
    pWin->AddPreRenderCallback( EstimateCameraPose, NULL );
    pWin->AddPostRenderCallback( ShowCameraAndTextures, NULL );

    return( pWin->Run( ));
}
