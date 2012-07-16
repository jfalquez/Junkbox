#include <SimpleGui/Gui.h>

#include <boost/thread.hpp>

#include <Mvlpp/SE3.h>

#include <opencv2/highgui/highgui.hpp>

#include "se3.h"
#include "so3.h"

#include "GLMosaic.h"
#include "GLHeightMap.h"
#include "PeaksHeightMap.h"


// Global Vars
GLImage glImgDiff;
int g_nImgWidth = 128;
int g_nImgHeight = 128;
bool g_DoESM = false;
bool g_Mutex;

// Global CVars
bool& g_bShowFrustum = CVarUtils::CreateCVar( "ShowFrustum", true, "Show camera viewing frustum." );

// Camera
GLSimCam Cam;
Eigen::Matrix4d g_dPose = GLCart2T( -30, -3, -1, 0, 0, 0 ); // initial camera pose
float g_fRoll = 0;
float g_fPitch = 0;
float g_fYaw = 0;
float g_fForward = 0;
float g_fRight = 0;
float g_fDown = 0;

// Reference Camera -- this is only used to show on screen estimated camera pose
GLSimCam VirtCam;
Eigen::Matrix4d g_dVirtPose = g_dPose;


/////////////////////////////////////////////////////////////////////////////////////////

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
				// forward
			case 'e': case 'E':
				g_fForward += 0.01;
				break;
			case 'q': case 'Q':
				g_fForward -= 0.01;
				break;
				// right
			case 'd': case 'D':
				g_fRight += 0.01;
				break;
			case 'a': case 'A':
				g_fRight -= 0.01;
				break;
				// down
			case 's': case 'S':
				g_fDown += 0.01;
				break;
			case 'w': case 'W':
				g_fDown -= 0.01;
				break;
				// pitch
			case 'i': case 'I':
				g_fPitch += 0.005;
				break;
			case 'k': case 'K':
				g_fPitch -= 0.005;
				break;
				// yaw
			case 'l': case 'L':
				g_fYaw += 0.005;
				break;
			case 'j': case 'J':
				g_fYaw -= 0.005;
				break;
				// roll
			case 'u': case 'U':
				g_fRoll -= 0.005;
				break;
			case 'o': case 'O':
				g_fRoll += 0.005;
				break;
			case ' ':
				g_fForward = 0;
				g_fRight = 0;
				g_fDown = 0;
				g_fRoll = 0;
				g_fPitch = 0;
				g_fYaw = 0;
				break;
			case 't': case 'T':
				g_DoESM = true;
				break;
			case 'r': case 'R':
				g_DoESM = false;
				g_dVirtPose = g_dPose;
				VirtCam.SetPose( g_dVirtPose );
				break;
			}
		}
		return SimpleDefaultEventHandler( e );
	}
};


/////////////////////////////////////////////////////////////////////////////////////////

Eigen::Vector2d _Project( const Eigen::Vector4d& P )
{

	// temporal working point
	Eigen::Vector3d T = P.block < 3, 1 > ( 0, 0 );

	// get camera intrinsics
	Eigen::Matrix3d K = Cam.GetKMatrix( );
	T = K * T;
	if( T( 2 ) == 0 ) {
		std::cout << "CRAP! " << T.transpose( ) << std::endl;
	}
	T = T / T( 2 );

	return T.block < 2, 1 > ( 0, 0 );
}


/////////////////////////////////////////////////////////////////////////////////////////

Eigen::Vector3d _BackProject( int X, int Y, double Depth )
{
	// get camera intrinsics
	Eigen::Matrix3d K = Cam.GetKMatrix( );
	double cx = K( 0, 2 );
	double cy = K( 1, 2 );
	double fx = K( 0, 0 );
	double fy = K( 1, 1 );

	Eigen::Vector3d P;
	// set into robotics coordinate frame
	P( 0 ) = Depth * ( ( X - cx ) / fx );
	P( 1 ) = Depth * ( ( Y - cy ) / fy );
	P( 2 ) = Depth;

	return P;
}


/////////////////////////////////////////////////////////////////////////////////////////

float _Interpolate(
		float x, //< Input: X coordinate
		float y, //< Input: Y coordinate
		const Eigen::VectorXf Image, //< Input: Image
		int ImageWidth = g_nImgWidth, //< Input: Image width
		int ImageHeight = g_nImgHeight //< Input: Image height
		)
{
	int xt = (int) x; /* top-left corner */
	int yt = (int) y;
	float ax = x - xt;
	float ay = y - yt;
	unsigned int idx = ( ImageWidth * yt ) + xt;

	if( xt >= 0 && yt >= 0 && xt <= ImageWidth - 2 && yt <= ImageHeight - 2 ) {
		return( ( 1 - ax ) * ( 1 - ay ) * Image[idx] +
				( ax ) * ( 1 - ay ) * Image[idx + 1] +
				( 1 - ax ) * ( ay ) * Image[idx + ImageWidth] +
				( ax ) * ( ay ) * Image[idx + ImageWidth + 1] );
	}
	return 0;
}


/////////////////////////////////////////////////////////////////////////////////////////

void _RGB2Gray(
		const std::vector < unsigned char >& RGB, //< Input: RGB image
		Eigen::VectorXf& Gray //< Output: Grayscale image
		)
{
	unsigned int Idx;
	for( int ii = 0; ii < g_nImgHeight; ii++ ) {
		for( int jj = 0; jj < g_nImgWidth; jj++ ) {
			// with flipping
			Idx = ( g_nImgHeight - ii - 1 ) * g_nImgWidth * 3 + jj * 3;
			Gray[ ii * g_nImgWidth + jj ] = float(RGB[Idx] + RGB[Idx + 1] + RGB[Idx + 2] ) / 3.0;

			// without flipping
			//Idx = ii * g_nImgWidth * 3 + jj * 3;
			//Gray[ii * g_nImgWidth + jj] = float(RGB[Idx] + RGB[Idx + 1] + RGB[Idx + 2] ) / 3.0;
		}
	}
}


/////////////////////////////////////////////////////////////////////////////////////////

void _WarpDepthMap( Eigen::VectorXf& vDM, Eigen::Vector6d Trf )
{
	// get rotation matrix and translation vector
	Eigen::Matrix3d R = mvl::Cart2R( Trf.block < 3, 1 > ( 0, 0 ) );
	Eigen::Vector3d T = Trf.block < 3, 1 > ( 3, 0 );


	Eigen::Vector3d P;
	for( int ii = 0; ii < g_nImgHeight; ii++ ) {
		for( int jj = 0; jj < g_nImgWidth; jj++ ) {
			P = _BackProject( jj, ii, vDM[ii * g_nImgWidth + jj] );
			P = R * P;
			P = P + T;
			vDM[ii * g_nImgWidth + jj] = P( 2 );
		}
	}
}


/////////////////////////////////////////////////////////////////////////////////////////

void UpdateCameraPose( GLWindow*, void* )
{
	g_dPose = g_dPose * GLCart2T( g_fForward, g_fRight, g_fDown, g_fRoll, g_fPitch, g_fYaw );
	Cam.SetPose( g_dPose );

	glEnable( GL_LIGHTING );
	glEnable( GL_LIGHT0 );
	glClearColor( 0.0, 0.0, 0.0, 1 );

	Cam.RenderToTexture( ); // will render to texture, then copy texture to CPU memory
	VirtCam.RenderToTexture( );

	g_Mutex = false;
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
	std::vector < unsigned char> vRGB; // original RGB images
	Eigen::VectorXf vImg; // grayscale image
	Eigen::VectorXf vVirtImg; // grayscale image
	Eigen::VectorXf vErrorImg; // grayscale image

	// resize vectors
	vImg.resize( g_nImgWidth * g_nImgHeight );
	vVirtImg.resize( g_nImgWidth * g_nImgHeight );

	// populate vectors
	Cam.CaptureRGB( vRGB );
	_RGB2Gray( vRGB, vImg );
	VirtCam.CaptureRGB( vRGB );
	_RGB2Gray( vRGB, vVirtImg );

	/*
	cv::Mat Tmp1( g_nImgHeight, g_nImgWidth, CV_8UC3, vRGB.data( ) );
	cv::imshow( "Img1", Tmp1 );
	vVirtImg = vVirtImg / 255;
	cv::Mat Tmp2( g_nImgHeight, g_nImgWidth, CV_32FC1, vVirtImg.data( ) );
	cv::imshow( "Img2", Tmp2 );
	cv::waitKey( 1 );

	for( int ii = 0; ii < 30; ii++ ) {
		printf("%d ", vRGB[(g_nImgWidth * 3) + ii]);
	}
	std::cout << std::endl << "Gray: " << std::endl;
	for( int ii = 0; ii < 30; ii++ ) {
		std::cout << vVirtImg[g_nImgWidth + ii] << " ";
	}
	/**/

	// calculate difference & normalize
	vErrorImg = ( vVirtImg - vImg ) / 255;

	glImgDiff.SetImage( (unsigned char*) vErrorImg.data( ), g_nImgWidth, g_nImgHeight, GL_LUMINANCE, GL_FLOAT );
	glImgDiff.SetSizeAsPercentageOfWindow( 0.66, 0.66, 1, 1 );
	DrawBorderAsWindowPercentage( 0.66, 0.66, 1, 1 );
	//printf( "\rImage Error: %f", vErrorImg.norm( ) );
	//fflush( stdout );
}

Eigen::Matrix4d convertToVisionFrame( Eigen::Matrix4d m )
{
	Eigen::Vector6d dVirtPose = mvl::T2Cart( m );
	double Temp;
	Temp = dVirtPose( 0 );
	dVirtPose( 0 ) = dVirtPose( 2 );
	dVirtPose( 2 ) = dVirtPose( 1 );
	dVirtPose( 1 ) = Temp;
	Temp = dVirtPose( 3 );
	dVirtPose( 3 ) = dVirtPose( 5 );
	dVirtPose( 5 ) = dVirtPose( 4 );
	dVirtPose( 4 ) = Temp;


	return mvl::Cart2T( dVirtPose );
}

Eigen::Matrix4d convertToRoboticsFrame( Eigen::Vector6d dVirtPose )
{
	//	Eigen::Vector6d dVirtPose = mvl::T2Cart( m );
	double Temp;
	Temp = dVirtPose( 0 );
	dVirtPose( 0 ) = dVirtPose( 1 );
	dVirtPose( 1 ) = dVirtPose( 2 );
	dVirtPose( 2 ) = Temp;
	Temp = dVirtPose( 3 );
	dVirtPose( 3 ) = dVirtPose( 4 );
	dVirtPose( 4 ) = dVirtPose( 5 );
	dVirtPose( 5 ) = Temp;

	return mvl::Cart2T( dVirtPose );
}
/////////////////////////////////////////////////////////////////////////////////////////

void ESM( )
{
	Eigen::Vector6d dVirtPose = mvl::T2Cart( g_dVirtPose );

	while( 1 ) {
		if( g_DoESM ) {
			std::vector < unsigned char> vRGB; // original RGB image
			Eigen::VectorXf vImg; // grayscale image
			Eigen::VectorXf vVirtImg; // grayscale image
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
			VirtCam.CaptureDepth( vVirtDepth.data( ) );

			// initial estimated pose is the last pose
			Eigen::Matrix4d dPrevPose = Eigen::Matrix4d::Identity( );
			Eigen::Vector6d dDeltaPose;

			Eigen::VectorXf ImgError;
			ImgError = vVirtImg - vImg;

			int nCount = 0;

			std::cout << "Real Pose: " << mvl::T2Cart( g_dPose ).transpose( ) << std::endl;
			std::cout << "Current Virtual Pose: " << mvl::T2Cart( g_dVirtPose ).transpose( ) << std::endl;

			while( /* ImgError.norm( ) */ 200 > 100.0 && nCount < 10 && g_DoESM ) {

				nCount++;

				// Jacobian
				Eigen::Matrix<double, 1, 6 > J;
				J = Eigen::Matrix<double, 1, 6 > ::Zero( );

				// LHS
				Eigen::Matrix<double, 6, 6 > LHS;
				LHS = Eigen::Matrix<double, 6, 6 > ::Zero( );

				// RHS
				Eigen::Matrix<double, 6, 1 > RHS;
				RHS = Eigen::Matrix<double, 6, 1 > ::Zero( );

				std::vector < Eigen::Matrix4d > Gen;
				Gen.resize( 6 );

				for( int ii = 0; ii < g_nImgHeight; ii++ ) {
					for( int jj = 0; jj < g_nImgWidth; jj++ ) {

						// "virtual" points in
						Eigen::Vector4d vP4; // homogeneous
						Eigen::Vector3d vP3; // 3d
						Eigen::Vector2d vP2; // 2d

						// check if point is contained in our model (i.e. has depth)
						if( vVirtDepth[ii * g_nImgWidth + jj] == 0 ) {
							continue;
						}

						//--------------------- first term 1x2

						// evaluate 'a' = L[ Tprev * Linv( u ) ]
						// back project from virtual camera's reference frame
						vP3 = _BackProject( jj, ii, vVirtDepth[ii * g_nImgWidth + jj] );

						// convert to homogeneous coordinate
						vP4 << vP3, 1;

						// transform point to real camera's reference frame
						// inv(Twr) * Twv * P
						vP4 = dPrevPose * vP4;

						// project into real camera's reference frame
						vP2 = _Project( vP4 );

						// check if point falls in camera's field of view
						if( vP2( 0 ) < 0 || vP2( 0 ) >= g_nImgWidth || vP2( 1 ) < 0 || vP2( 1 ) >= g_nImgHeight ) {
							continue;
						}

						// finite differences
						float TopPix = _Interpolate( vP2( 0 ), vP2( 1 ) - 1, vImg );
						float BotPix = _Interpolate( vP2( 0 ), vP2( 1 ) + 1, vImg );
						float LeftPix = _Interpolate( vP2( 0 ) - 1, vP2( 1 ), vImg );
						float RightPix = _Interpolate( vP2( 0 ) + 1, vP2( 1 ), vImg );

						Eigen::Matrix<double, 1, 2 > Term1;
						Term1( 0 ) = ( RightPix - LeftPix ) / 2;
						Term1( 1 ) = ( TopPix - BotPix ) / 2;

						//--------------------- second term 2x3

						// evaluate 'b' = Tprev * Linv( u )
						// point already stored in vP4

						// fill matrix
						// 1/c      0       -a/c^2
						//  0       1/c     -b/c^2
						Eigen::Matrix< double, 2, 3 > Term2;
						// the projection function already de-homogenizes therefore c = 1
						Term2( 0, 0 ) = 1;
						Term2( 0, 1 ) = 0;
						Term2( 0, 2 ) = -vP2( 0 );
						Term2( 1, 0 ) = 0;
						Term2( 1, 1 ) = 1;
						Term2( 1, 2 ) = -vP2( 1 );


						//--------------------- third term 3x1
						Eigen::Vector4d Term3i;
						// last row of Term3 is truncated since it is always 1
						Eigen::Vector3d Term3;

						// fill Jacobian with T generators

						// 1st generator
						Gen[0] << 0, 0, 0, 1,
								0, 0, 0, 0,
								0, 0, 0, 0,
								0, 0, 0, 0;
						Term3i = mvl::TInv( dPrevPose ) * Gen[0] * vP4;
						Term3 = Term3i.block < 3, 1 > ( 0, 0 );
						J( 0, 0 ) = Term1 * Term2 * Term3;
//						J( 0, 0 ) = Term3;

						// 2nd generator
						Gen[1] << 0, 0, 0, 0,
								0, 0, 0, 1,
								0, 0, 0, 0,
								0, 0, 0, 0;
						Term3i = mvl::TInv( dPrevPose ) * Gen[1] * vP4;
						Term3 = Term3i.block < 3, 1 > ( 0, 0 );
						J( 0, 1 ) = Term1 * Term2 * Term3;
//						J( 0, 1 ) = Term3;

						// 3rd generator
						Gen[2] << 0, 0, 0, 0,
								0, 0, 0, 0,
								0, 0, 0, 1,
								0, 0, 0, 0;
						Term3i = mvl::TInv( dPrevPose ) * Gen[2] * vP4;
						Term3 = Term3i.block < 3, 1 > ( 0, 0 );
						J( 0, 2 ) = Term1 * Term2 * Term3;
//						J( 0, 2 ) = Term3;

						// 4th generator
						Gen[3] << 0, 0, 0, 0,
								0, 0, -1, 0,
								0, 1, 0, 0,
								0, 0, 0, 0;
						Term3i = mvl::TInv( dPrevPose ) * Gen[3] * vP4;
						Term3 = Term3i.block < 3, 1 > ( 0, 0 );
						J( 0, 3 ) = Term1 * Term2 * Term3;
//						J( 0, 3 ) = Term3;

						// 5th generator
						Gen[4] << 0, 0, 1, 0,
								0, 0, 0, 0,
								-1, 0, 0, 0,
								0, 0, 0, 0;
						Term3i = mvl::TInv( dPrevPose ) * Gen[4] * vP4;
						Term3 = Term3i.block < 3, 1 > ( 0, 0 );
						J( 0, 4 ) = Term1 * Term2 * Term3;
//						J( 0, 4 ) = Term3;

						// 6th generator
						Gen[5] << 0, -1, 0, 0,
								1, 0, 0, 0,
								0, 0, 0, 0,
								0, 0, 0, 0;
						Term3i = mvl::TInv( dPrevPose ) * Gen[5] * vP4;
						Term3 = Term3i.block < 3, 1 > ( 0, 0 );
						J( 0, 5 ) = Term1 * Term2 * Term3;
//						J( 0, 5 ) = Term3;

//						Eigen::Matrix4d mFD;
//						mFD = ((dPrevPose * mvl::Cart2T(g_Delta)) - (dPrevPose * mvl::Cart2T(-v6A))) / 2;
//						std::cout << mFD << std::endl;



						// estimate LHS (Hessian)
						// LHS = Hessian = Jt * J
						LHS += J.transpose( ) * J;
						//						std::cout << "J is: " << std::endl << J << std::endl;

						// estimate RHS (error)
						// RHS = Jt * e
						RHS += J.transpose( ) * ( vVirtImg[ii * g_nImgWidth + jj] - vImg[ii * g_nImgWidth + jj] );
					}
				}
				std::cout << "LHS is: " << std::endl << LHS.transpose( ) << std::endl;
				std::cout << "RHS is: " << RHS.transpose( ) << std::endl;

				// calculate deltaPose as Hinv * Jt * error
				//deltaPose = LHS.inverse() * RHS;
				dDeltaPose = LHS.ldlt( ).solve( RHS );

				// convert deltaPose from "Lie" to "Cartesian" =(
				Sophus::SE3 deltaPoseT;
				deltaPoseT = Sophus::SE3::exp( dDeltaPose );

				std::cout << "Delta pose is: " << dDeltaPose.transpose( ) << std::endl;

				dPrevPose = dPrevPose * mvl::Cart2T(-dDeltaPose);

				//VirtCam.SetPose( g_dVirtPose * convertToRoboticsFrame( deltaPose ) );
				VirtCam.SetPose( g_dVirtPose * convertToRoboticsFrame( dDeltaPose ) );
				std::cout << "New Virtual Pose is: " << mvl::T2Cart( g_dVirtPose * convertToRoboticsFrame( -dDeltaPose ) ).transpose() << std::endl;

				// we cannot RenderToTexture() here since it is a different thread
				// we need to synchronize with the GUI so that the Virtual Camera is
				// rendered at the new pose we just updated.
				//VirtCam.RenderToTexture( );
				g_Mutex = true;
				while( g_Mutex ) {
				}

				VirtCam.CaptureRGB( vRGB );
				_RGB2Gray( vRGB, vVirtImg );
				VirtCam.CaptureDepth( vVirtDepth.data( ) );


				// calculate new error
				ImgError = vVirtImg - vImg;
				std::cout << "Error is: " << ImgError.norm( ) << std::endl;
			}

			std::cout << "Real Pose: " << mvl::T2Cart( g_dPose ).transpose( ) << std::endl;
			std::cout << "Estimated Pose: " << (mvl::T2Cart( g_dVirtPose ) + dDeltaPose).transpose( ) << std::endl;
			g_DoESM = false;
		}
	}
}


/////////////////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
	// register CVars
	//    CVarUtils::AttachCVar( "CamPose", &g_dPose );

	// parse arguments
	GetPot cl( argc, argv );
	std::string sMesh = cl.follow( "antoine.obj", "-mesh" );

	// init window
	GuiWindow* pWin = new GuiWindow( 0, 0, 1024, 768, "Dense ESM" );

	//	GLGrid glGrid;
	//	glGrid.SetPerceptable( false );

	//	GLMesh glMesh;
	//	glMesh.Init( sMesh );

	GLMosaic glMosaic;
	glMosaic.Init( 200, 200 );
	Eigen::Matrix4d BasePose;
	BasePose << 1, 0, 0, 10,
			0, 1, 0, -100,
			0, 0, 1, -100,
			0, 0, 0, 1;
	glMosaic.SetBaseFrame( BasePose );

	//	PeaksHeightMap glPHM;
	//	GLHeightMap glHM( &glPHM );

	glImgDiff.InitReset( );
	glImgDiff.SetVisible( );
	glImgDiff.SetPerceptable( false );

	// register objects
	//	pWin->AddChildToRoot( &glHM );
	pWin->AddChildToRoot( &glMosaic );
	//	pWin->AddChildToRoot( &glMesh );
	//	pWin->AddChildToRoot( &glGrid );
	pWin->AddChildToRoot( &glImgDiff );

	//	glMesh.SetPerceptable( true );
	//	glMesh.SetVisible();
	//	glMesh.SetPose( 0, 0, -1, 0, 0, 0 );

	Eigen::Matrix3d dK; // computer vision K matrix
	dK << g_nImgWidth, 0, g_nImgWidth / 2,
			0, g_nImgHeight, g_nImgHeight / 2,
			0, 0, 1;

	Cam.Init( &pWin->SceneGraph( ), g_dPose, dK, g_nImgWidth, g_nImgHeight, eSimCamRGB );
	VirtCam.Init( &pWin->SceneGraph( ), g_dVirtPose, dK, g_nImgWidth, g_nImgHeight, eSimCamDepth | eSimCamRGB );

	glEnable( GL_LIGHT0 ); // activate light0
	glEnable( GL_LIGHTING ); // enable lighting

	pWin->LookAt( -50, 10, -10, 0, 10, 0, 0, 0, -1 );


	// add our callbacks
	pWin->AddPreRenderCallback( UpdateCameraPose, NULL );
	pWin->AddPostRenderCallback( ShowCameraAndTextures, NULL );

	// need to launch ESM thread
	boost::thread ESM_Thread( ESM );

	return( pWin->Run( ) );
}