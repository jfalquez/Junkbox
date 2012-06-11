#include <SimpleGui/Gui.h>

//#include <CVarHelpers.h>

#include <Mvlpp/SE3.h>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "GLHeightMap.h"
#include "PeaksHeightMap.h"


// Global Vars
GLImage				glImgDiff;
int					g_nImgWidth = 128;
int					g_nImgHeight = 128;
bool				g_DoESM = false;

// Global CVars
bool& g_bShowFrustum = CVarUtils::CreateCVar( "ShowFrustum", true, "Show camera viewing frustum." );

// Camera
GLSimCam Cam;
Eigen::Matrix4d		g_dPose = GLCart2T( -40, 0, -5, 0, 0, 0 ); // initial camera pose
//Eigen::Matrix4d g_dPose = GLCart2T( 0, 0, -40, 0, -M_PI / 2, 0 ); // initial camera pose
float				g_fTurnrate = 0;
float				g_fSpeed = 0;

// Reference Camera -- this is only used to show on screen estimated camera pose
GLSimCam			VirtCam;
Eigen::Matrix4d		g_dVirtPose = g_dPose;


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
                    g_fSpeed -= 0.1;
                    break;
                case 'd': case 'D':
                    g_fTurnrate += 0.01;
                    break;
                case 'w': case 'W':
                    g_fSpeed += 0.1;
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
	VirtCam.RenderToTexture( );
}

/////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector2d _Project( const Eigen::Vector4d& P ) {

	// temporal working point
    Eigen::Vector3d T = P.block<3,1>(0,0);

	// get camera intrinsics
    Eigen::Matrix3d K = Cam.GetKMatrix();
	T = K * T;

    return T.block<2,1>(0,0);
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
float _Interpolate(
                float x,						//< Input: X coordinate
                float y,						//< Input: Y coordinate
                const Eigen::VectorXf Image,	//< Input: Image
                int ImageWidth = g_nImgWidth,	//< Input: Image width
                int ImageHeight = g_nImgHeight	//< Input: Image height
                )
{
    int xt = (int) x;  /* top-left corner */
    int yt = (int) y;
    float ax = x - xt;
    float ay = y - yt;
    unsigned int idx = (ImageWidth*yt) + xt;

    if (xt >= 0 && yt >= 0 && xt <= ImageWidth - 2 && yt <= ImageHeight - 2) {
	    return ( (1-ax) * (1-ay) * Image[idx] +
                        ( ax ) * (1-ay) * Image[idx+1] +
                        (1-ax) * ( ay ) * Image[idx+ImageWidth] +
                        ( ax ) * ( ay ) * Image[idx+ImageWidth+1] );
	}
	return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
void _RGB2Gray(
                const std::vector < unsigned char >& RGB,		//< Input: RGB image
                Eigen::VectorXf& Gray							//< Output: Grayscale image
                )
{
	unsigned int Idx;
	for( int ii = 0; ii < g_nImgHeight; ii++ ) {
		for ( int jj = 0; jj < g_nImgWidth; jj++ ) {
			Idx = (g_nImgHeight-ii-1)*g_nImgWidth*3 + jj*3;
			Gray[ ii*g_nImgWidth + jj ] = float(RGB[Idx] + RGB[Idx+1] + RGB[Idx+2]) / 3.0;
		}
	}
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
	std::vector < unsigned char>  vRGB;		// original RGB image
    Eigen::VectorXf vImg;					// grayscale image
    Eigen::VectorXf vVirtImg;				// grayscale image
    Eigen::VectorXf vVirtDepth;

    // resize vectors
    vRGB.resize( g_nImgWidth * g_nImgHeight * 3 );
    vImg.resize( g_nImgWidth * g_nImgHeight );
    vVirtImg.resize( g_nImgWidth * g_nImgHeight );
    vVirtDepth.resize( g_nImgWidth * g_nImgHeight );

    // populate vectors & convert to grayscale
	Cam.CaptureRGB( vRGB );
	_RGB2Gray( vRGB, vImg );
    VirtCam.CaptureRGB( vRGB );
	_RGB2Gray( vRGB, vVirtImg );

    /*
	cv::Mat Tmp1( g_nImgHeight, g_nImgWidth, CV_8UC3, vRGB.data() );
	cv::imshow( "Img1", Tmp1 );
	cv::Mat Tmp2( g_nImgHeight, g_nImgWidth, CV_32FC1, vImg.data() );
	cv::imshow( "Img2", Tmp2 );
	cv::waitKey(100);
    /**/

	// assuming depth is not normalized
//    VirtCam.CaptureDepth( vVirtDepth.data() );
    VirtCam.CaptureUnnormalizedDepth( vVirtDepth.data() );

    /*
    // check to see if the depth cam works...
    std::cout << "Size: " << vVirtDepth.size( ) << std::endl;
    for( int ii = 0; ii < g_nImgHeight; ii++ ) {
        for( int jj = 0; jj < g_nImgWidth; jj++ ) {
         printf( "%.2f ", vVirtDepth[ii*g_nImgWidth+jj] );
        }
		printf("\n");
    }
    /**/

    // initial estimated pose is the last pose
    Eigen::Vector6d dPose = Eigen::Vector6d::Zero();

    Eigen::VectorXf ImgError;
    ImgError = vVirtImg - vImg;

	int nCount = 0;
    while( ImgError.norm() > 1.0  && nCount < 10 ) {
		nCount++;

        // Jacobian
        Eigen::Matrix<double,1,6> J;

        // LHS
        Eigen::Matrix<double,6,6> LHS;
        LHS = Eigen::Matrix<double,6,6>::Zero();

        // RHS
        Eigen::Matrix<double,6,1> RHS;
        RHS = Eigen::Matrix<double,6,1>::Zero();

        for( int ii = 0; ii < g_nImgHeight; ii++ ) {
            for( int jj = 0; jj < g_nImgWidth; jj++ ) {

				// "virtual" points in
				Eigen::Vector4d vP4;
                Eigen::Vector3d vP3;
				Eigen::Vector2d vP2;

				//--------------------- first term 1x2

                // evaluate 'a' = L[ Tprev * Linv( u ) ]
                vP3 = _BackProject( jj, ii, vVirtDepth[ii*g_nImgWidth + jj] );
				vP4 << vP3, 1; // homogeneous coordinate
				vP4 = mvl::Cart2T( dPose ) * vP4;
				vP2 = _Project( vP4 );

				// finite differences
				float TopPix = _Interpolate( vP2(0), vP2(1)-1, vVirtImg );
				float BotPix = _Interpolate( vP2(0), vP2(1)+1, vVirtImg );
				float LeftPix = _Interpolate( vP2(0)-1, vP2(1), vVirtImg );
				float RightPix = _Interpolate( vP2(0)+2, vP2(1), vVirtImg );

				Eigen::Matrix<double, 1, 2> Term1;
				Term1(0) = (RightPix - LeftPix) / 2;
				Term1(1) = (TopPix - BotPix) / 2;

				//--------------------- second term 2x3

                // evaluate 'b' = Tprev * Linv( u )
                // already stored in vP4
                //vP3 = _BackProject( jj, ii, vVirtDepth[ii*g_nImgWidth + jj] );
				//vP4 << vP3, 1; // homogeneous coordinate
				//vP4 = mvl::Cart2T( dPose ) * vP4;

                // fill matrix
                // 1/c      0       -a/c^2
                //  0       1/c     -b/c^2
                Eigen::Matrix< double, 2, 3 > Term2;
                Term2( 0, 0 ) = 1 / vP4(2);
                Term2( 0, 1 ) = 0;
                Term2( 0, 2 ) = - vP4(0) / vP4(2) * vP4(2);
                Term2( 1, 0 ) = 0;
                Term2( 1, 1 ) = 1 / vP4(2);
                Term2( 1, 2 ) = - vP4(1) / vP4(2) * vP4(2);


				//--------------------- third term 3x1
                Eigen::Vector4d Term3i;
                // last row of Term3 is truncated since it is always 1
                Eigen::Vector3d Term3;

                //vP3 = _BackProject( jj, ii, vVirtDepth[ii*g_nImgWidth + jj] );
				vP4 << vP3, 1; // homogeneous coordinate

                Eigen::Matrix4d Gen;
                // fill Jacobian with T generators

                // 1st generator
                Gen << 0, 0, 0, 1,
                       0, 0, 0, 0,
                       0, 0, 0, 0,
                       0, 0, 0, 0;
                Term3i = mvl::Cart2T( dPose ) * Gen * vP4;
                Term3 = Term3i.block<3,1>(0,0);
                J( 0, 0 ) = Term1 * Term2 * Term3;

                // 2nd generator
                Gen << 0, 0, 0, 0,
                       0, 0, 0, 1,
                       0, 0, 0, 0,
                       0, 0, 0, 0;
                Term3i = mvl::Cart2T( dPose ) * Gen * vP4;
                Term3 = Term3i.block<3,1>(0,0);
                J( 0, 1 ) = Term1 * Term2 * Term3;

                // 3rd generator
                Gen << 0, 0, 0, 0,
                       0, 0, 0, 0,
                       0, 0, 0, 1,
                       0, 0, 0, 0;
                Term3i = mvl::Cart2T( dPose ) * Gen * vP4;
                Term3 = Term3i.block<3,1>(0,0);
                J( 0, 2 ) = Term1 * Term2 * Term3;

                // 4th generator
                Gen << 0, 0, 0, 0,
                       0, 0, 1, 0,
                       0, -1, 0, 0,
                       0, 0, 0, 0;
                Term3i = mvl::Cart2T( dPose ) * Gen * vP4;
                Term3 = Term3i.block<3,1>(0,0);
                J( 0, 3 ) = Term1 * Term2 * Term3;

                // 5th generator
                Gen << 0, 0, -1, 0,
                       0, 0, 0, 0,
                       1, 0, 0, 0,
                       0, 0, 0, 0;
                Term3i = mvl::Cart2T( dPose ) * Gen * vP4;
                Term3 = Term3i.block<3,1>(0,0);
                J( 0, 4 ) = Term1 * Term2 * Term3;

                // 6th generator
                Gen << 0, 1, 0, 0,
                       -1, 0, 0, 0,
                       0, 0, 0, 0,
                       0, 0, 0, 0;
                Term3i = mvl::Cart2T( dPose ) * Gen * vP4;
                Term3 = Term3i.block<3,1>(0,0);
                J( 0, 5 ) = Term1 * Term2 * Term3;

                // estimate LHS (Hessian)
                // LHS = Hessian = Jt * J
                LHS += J.transpose() * J;

                // estimate RHS (error)
                // RHS = Jt * e
//                RHS += J.transpose() * (Pe - P);
            }
        }

        // calculate deltaPose as Hinv * Jt * error
        Eigen::Vector6d deltaPose;
//        deltaPose = LHS.inverse() * RHS;
//        deltaPose = LHS.ldlt().solve(RHS);


        // update dPose += deltaPose
        dPose += deltaPose;

        // warp
//        _WarpDepthMap( vVirtImg, deltaPose );

        // move camera to new pose and re-render
		g_dVirtPose = g_dVirtPose * mvl::Cart2T( dPose );
        VirtCam.SetPose( g_dVirtPose );
        VirtCam.RenderToTexture( );

        // calculate new error
        ImgError = vVirtImg - vImg;
        std::cout << "Error is: " << ImgError.norm() << std::endl;
    }

    std::cout << "Real Pose: " << mvl::T2Cart(g_dPose).transpose() << std::endl;
    std::cout << "Estimated Pose: " << mvl::T2Cart(g_dVirtPose).transpose() << std::endl;
    g_DoESM = false;

    }
}


/////////////////////////////////////////////////////////////////////////////////////////
void ShowCameraAndTextures( GLWindow*, void* )
{
    if( g_bShowFrustum ) {
        // show the camera
        Cam.DrawCamera( );
        VirtCam.DrawCamera( );
    }

    /// show textures
    if( Cam.HasRGB( ) ) {
        DrawTextureAsWindowPercentage( Cam.RGBTexture( ), Cam.ImageWidth( ),
                Cam.ImageHeight( ), 0, 0.66, 0.33, 1 );
        DrawBorderAsWindowPercentage( 0, 0.66, 0.33, 1 );
    }

    if( VirtCam.HasRGB( ) ) {
        DrawTextureAsWindowPercentage( VirtCam.RGBTexture( ), VirtCam.ImageWidth( ),
                VirtCam.ImageHeight( ), 0.33, 0.66, 0.66, 1 );
        DrawBorderAsWindowPercentage( 0.33, 0.66, 0.66, 1 );
    }

    // draw difference image
//    Eigen::Matrix<unsigned char, 1, Eigen::Dynamic> vImg;
//    Eigen::Matrix<unsigned char, 1, Eigen::Dynamic> vVirtImg;
	std::vector < unsigned char>  vRGB;		// original RGB image
    Eigen::VectorXf vImg;				// grayscale image
    Eigen::VectorXf vVirtImg;				// grayscale image
    Eigen::VectorXf vErrorImg;				// grayscale image
//    Eigen::Matrix<unsigned char, 1, Eigen::Dynamic> vErrorImg;

    // resize vectors
    vImg.resize( g_nImgWidth * g_nImgHeight );
    vVirtImg.resize( g_nImgWidth * g_nImgHeight );

    // populate vectors
//    Cam.CaptureRGB( vImg.data() );
//    VirtCam.CaptureRGB( vVirtImg.data() );
	Cam.CaptureRGB( vRGB );
	_RGB2Gray( vRGB, vImg );
    VirtCam.CaptureRGB( vRGB );
	_RGB2Gray( vRGB, vVirtImg );

    // calculate difference
    vErrorImg = vVirtImg - vImg;

//    glImgDiff.SetImage( vErrorImg.data(), g_nImgWidth, g_nImgHeight, GL_LUMINANCE, GL_UNSIGNED_BYTE );
    glImgDiff.SetSizeAsPercentageOfWindow( 0.66, 0.66, 1, 1);
    DrawBorderAsWindowPercentage( 0.66, 0.66, 1, 1 );
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
	glImgDiff.SetVisible();
    glImgDiff.SetPerceptable( false );

    // register objects
    pWin->AddChildToRoot( &glHM );
    pWin->AddChildToRoot( &glGrid );
    pWin->AddChildToRoot( &glImgDiff );

	Eigen::Matrix3d	dK; // computer vision K matrix
    dK <<   g_nImgWidth,    0,              g_nImgWidth / 2,
            0,              g_nImgHeight,   g_nImgHeight / 2,
            0,              0,              1;

    Cam.Init( &pWin->SceneGraph( ), g_dPose, dK, g_nImgWidth, g_nImgHeight, eSimCamRGB );
    VirtCam.Init( &pWin->SceneGraph( ), g_dVirtPose, dK, g_nImgWidth, g_nImgHeight, eSimCamDepth | eSimCamRGB );

    glEnable( GL_LIGHT0 ); // activate light0
    glEnable( GL_LIGHTING ); // enable lighting

    pWin->LookAt( -70, -70, -50, 0, 0, 0, 0, 0, -1 );


    // add our callbacks
    pWin->AddPreRenderCallback( UpdateCameraPose, NULL );
    pWin->AddPreRenderCallback( EstimateCameraPose, NULL );
    pWin->AddPostRenderCallback( ShowCameraAndTextures, NULL );

    return( pWin->Run( ));
}
