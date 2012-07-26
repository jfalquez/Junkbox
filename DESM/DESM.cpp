#include <SimpleGui/Gui.h>

#include <boost/thread.hpp>

#include <Mvlpp/Mvl.h>

#include <opencv2/highgui/highgui.hpp>

#include "se3.h"
#include "so3.h"

#include "GLMosaic.h"
#include "GLImgPlane.h"
#include "GLHeightMap.h"
#include "PeaksHeightMap.h"

// GL Objects
GLImage glImgDiff;
GLMosaic glMosaic;
Eigen::Vector6d MosPos = Eigen::Vector6d::Zero();


// Global CVars
bool& g_bShowFrustum = CVarUtils::CreateCVar( "ShowFrustum", true, "Show camera viewing frustum." );


// Global Vars
int g_nImgWidth = 128;
int g_nImgHeight = 128;
bool g_DoESM = false;
bool g_DoGrDescent = false;
bool g_Mutex;


// Camera
GLSimCam RefCam; // reference camera we move
GLSimCam VirtCam; // virtual camera which calculates transformation
GLSimCam GDCam;

// Camera Poses
Eigen::Matrix4d g_dRefPose = mvl::Cart2T( -29.5, 0, 0, 0, 0, 0 ); // initial camera pose
Eigen::Matrix4d g_dVirtPose = mvl::Cart2T( -30, 0, 0, 0, 0, 0 ); // initial camera pose
//Eigen::Matrix4d g_dVirtPose = g_dRefPose;


// Reference Camera Controls
Eigen::Vector6d g_dRefVel = Eigen::Vector6d::Zero();



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
				g_dRefVel(0) += 0.01;
				break;
			case 'q': case 'Q':
				g_dRefVel(0) -= 0.01;
				break;
				// right
			case 'd': case 'D':
				g_dRefVel(1) += 0.01;
				break;
			case 'a': case 'A':
				g_dRefVel(1) -= 0.01;
				break;
				// down
			case 's': case 'S':
				g_dRefVel(2) += 0.01;
				break;
			case 'w': case 'W':
				g_dRefVel(2) -= 0.01;
				break;
				// pitch
			case 'i': case 'I':
				g_dRefVel(4) += 0.005;
				break;
			case 'k': case 'K':
				g_dRefVel(4) -= 0.005;
				break;
				// yaw
			case 'l': case 'L':
				g_dRefVel(5) += 0.005;
				break;
			case 'j': case 'J':
				g_dRefVel(5) -= 0.005;
				break;
				// roll
			case 'u': case 'U':
				g_dRefVel(3) -= 0.005;
				break;
			case 'o': case 'O':
				g_dRefVel(3) += 0.005;
				break;
			case ' ':
				g_dRefVel << 0,0,0,0,0,0;
				break;
			case 't': case 'T':
				g_DoESM = true;
				break;
			case 'g': case 'G':
				g_DoGrDescent = true;
				break;
			case 'r': case 'R':
				g_DoESM = false;
				g_dVirtPose = g_dRefPose;
				VirtCam.SetPose( g_dVirtPose );
				break;
			}
		}
		return SimpleDefaultEventHandler( e );
	}
};


/////////////////////////////////////////////////////////////////////////////////////////

Eigen::Vector3d _Project( const Eigen::Vector3d& P )
{
	Eigen::Vector3d T = P;

	// permutation matrix
	Eigen::Matrix3d M;
	M << 0, 1, 0,
		 0, 0, 1,
		 1, 0, 0;

	// get camera intrinsics
	Eigen::Matrix3d K = RefCam.GetKMatrix( );
	K = K * M;
	T = K * T;
	if( T( 2 ) == 0 ) {
		std::cout << "CRAP! " << T.transpose( ) << std::endl;
	}

	return T;
}


/////////////////////////////////////////////////////////////////////////////////////////

Eigen::Vector3d _BackProject( int X, int Y, double Depth )
{
	// get camera intrinsics
	Eigen::Matrix3d K = RefCam.GetKMatrix( );

	double cx = K( 0, 2 );
	double cy = K( 1, 2 );
	double fx = K( 0, 0 );
	double fy = K( 1, 1 );

	Eigen::Vector3d P;

	P( 1 ) = Depth * ( ( X - cx ) / fx );
	P( 2 ) = Depth * ( ( Y - cy ) / fy );
	P( 0 ) = Depth;

	return P;
}


/////////////////////////////////////////////////////////////////////////////////////////

Eigen::Vector3d _BackProject2( int X, int Y, double Depth )
{
	// get camera intrinsics
	Eigen::Matrix3d K = RefCam.GetKMatrix( );

	// permutation matrix
	Eigen::Matrix3d M;
	M << 0, 1, 0,
		 0, 0, 1,
		 1, 0, 0;

	K = K * M;

	Eigen::Vector3d P;

	P << X, Y, 1;

	P = P * Depth;

	P = K.inverse() * P;

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

void _FlipDepth(
		Eigen::VectorXf& vDepth //< Input/Output: Depth buffer
		)
{
	Eigen::VectorXf tmp;
	tmp = vDepth;

	unsigned int Idx;
	for( int ii = 0; ii < g_nImgHeight; ii++ ) {
		for( int jj = 0; jj < g_nImgWidth; jj++ ) {
			Idx = ( g_nImgHeight - ii - 1 ) * g_nImgWidth + jj;
			vDepth[ ii * g_nImgWidth + jj ] = tmp[ Idx ];
		}
	}
}


/////////////////////////////////////////////////////////////////////////////////////////

void UpdateCameraPose( GLWindow*, void* )
{
	//MosPos += g_dRefVel;
	//glMosaic.SetBaseFrame( mvl::Cart2T( MosPos ));

	g_dRefPose = g_dRefPose * mvl::Cart2T( g_dRefVel );
	RefCam.SetPose( g_dRefPose );

	glEnable( GL_LIGHTING );
	glEnable( GL_LIGHT0 );
	glClearColor( 0.0, 0.0, 0.0, 1 );

	RefCam.RenderToTexture( ); // will render to texture, then copy texture to CPU memory
	VirtCam.RenderToTexture( );

	g_Mutex = false;
}


/////////////////////////////////////////////////////////////////////////////////////////

void ShowCameraAndTextures( GLWindow*, void* )
{
	if( g_bShowFrustum ) {
		// show the camera
		RefCam.DrawCamera( );
		VirtCam.DrawCamera( );
	}

	/// show textures
	if( RefCam.HasRGB( ) ) {
		DrawTextureAsWindowPercentage( RefCam.RGBTexture( ), RefCam.ImageWidth( ),
				RefCam.ImageHeight( ), 0, 0.66, 0.33, 1 );
		DrawBorderAsWindowPercentage( 0, 0.66, 0.33, 1 );
	}

	if( VirtCam.HasRGB( ) ) {
		DrawTextureAsWindowPercentage( VirtCam.RGBTexture( ), VirtCam.ImageWidth( ),
				VirtCam.ImageHeight( ), 0.33, 0.66, 0.66, 1 );
		DrawBorderAsWindowPercentage( 0.33, 0.66, 0.66, 1 );
	}

	// draw difference image
	std::vector < unsigned char> vRGB; // original RGB images
	Eigen::VectorXf vRefImg; // grayscale image
	Eigen::VectorXf vVirtImg; // grayscale image
	Eigen::VectorXf vErrorImg; // grayscale image

	// resize vectors
	vRefImg.resize( g_nImgWidth * g_nImgHeight );
	vVirtImg.resize( g_nImgWidth * g_nImgHeight );

	// populate vectors
	RefCam.CaptureRGB( vRGB );
	_RGB2Gray( vRGB, vRefImg );
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

	// calculate error
	vErrorImg = vRefImg - vVirtImg;


	// stop refinement if error is low enough
	if( vErrorImg.norm( ) < 100 ) {
		g_DoGrDescent = false;
	}


	// normalize error
	vErrorImg = vErrorImg.array().abs();
	if( vErrorImg.maxCoeff() != 0 ) {
		vErrorImg = vErrorImg / vErrorImg.maxCoeff();
	}

	/*
	// Point in in world (robotic) reference frame
	Eigen::Vector2d U;
	Eigen::Vector3d P3;
	Eigen::Vector4d P3h;

	P3h << MosPos.head(3), 1;
	P3h = mvl::TInv(g_dRefPose) * P3h;

	P3 = _Project( P3h.head(3) );
	U = P3.head(2);
	U = U / P3(2);

	unsigned int idx = (int(U(1)) * g_nImgWidth) + int(U(0));
	vErrorImg = vRefImg - vRefImg;
	//vErrorImg[ idx ] = 1.0;
	/* */

	/*
	Eigen::VectorXf vVirtDepth; // grayscale image
	vVirtDepth.resize( g_nImgWidth * g_nImgHeight );
	VirtCam.CaptureDepth( vVirtDepth.data( ) );
	_FlipDepth( vVirtDepth );

	for( int ii = 0; ii < vVirtDepth.size(); ii++ ) {
		if( vVirtDepth[ii] != 0 ) {
			std::cout << "Found something at " << ii << " " << vVirtDepth[ii] << std::endl;
			idx = ii;
		}
	}
	U(1) = floor(idx / g_nImgWidth);
	U(0) = idx - (U(1) * g_nImgWidth);

	P3 = _BackProject( U(0), U(1), vVirtDepth[idx] );
	P3h << P3, 1;
	P3h = mvl::TInv(g_dRefPose) * g_dVirtPose * P3h;
	P3 = _Project( P3h.head(3) );
	U = P3.head(2);
	U = U / P3(2);
	idx = (int(U(1)) * g_nImgWidth) + int(U(0));
	vErrorImg[ idx ] = 1.0;
	/* */

//	for( float ii = -1.0; ii < 1.0; ii += 0.1 ) {
//		for( float jj = -1.0; jj < 1.0; jj += 0.1 ) {
//
//		}
//	}


	glImgDiff.SetImage( (unsigned char*) vErrorImg.data( ), g_nImgWidth, g_nImgHeight, GL_LUMINANCE, GL_FLOAT );
	glImgDiff.SetSizeAsPercentageOfWindow( 0.66, 0.66, 1, 1 );
	DrawBorderAsWindowPercentage( 0.66, 0.66, 1, 1 );
//	printf( "\rNormalized Image Error: %f", vErrorImg.norm( ) );
//	fflush( stdout );
}


/////////////////////////////////////////////////////////////////////////////////////////

void DoPoseRefinement( GLWindow*, void* )
{
	if( g_DoGrDescent ) {
		// image holders
		std::vector < unsigned char> vRGB; // original RGB image
		Eigen::VectorXf vRefImg; // grayscale image
		Eigen::VectorXf vVirtImg; // grayscale image

		// resize vectors
		vRGB.resize( g_nImgWidth * g_nImgHeight * 3 );
		vRefImg.resize( g_nImgWidth * g_nImgHeight );
		vVirtImg.resize( g_nImgWidth * g_nImgHeight );

		// populate vectors & convert to grayscale
		RefCam.CaptureRGB( vRGB );
		_RGB2Gray( vRGB, vRefImg );
		VirtCam.CaptureRGB( vRGB );
		_RGB2Gray( vRGB, vVirtImg );

		// camera for finite differences
		Eigen::VectorXf vGDImg;
		vGDImg.resize( g_nImgWidth * g_nImgHeight );


		// finite difference deltas
		float Tdelta = 0.01;
		float Rdelta = 0.005;

		// movement delta vector
		Eigen::Vector6d DeltaMv;

		// best movement & score
		double Error;
		double BestError;
		Eigen::Vector6d BestMv;

		//----------------------------------

		// forward
		DeltaMv << Tdelta, 0, 0, 0, 0, 0;
		BestMv = DeltaMv;
		GDCam.SetPose( g_dVirtPose * mvl::Cart2T( DeltaMv ) );
		GDCam.RenderToTexture( );
		GDCam.CaptureRGB( vRGB );
		_RGB2Gray( vRGB, vGDImg );
//		BestError = ( vRefImg - vFDImg ).norm( );
		BestError = ( vRefImg - vGDImg ).lpNorm<1>( );

		// back
		DeltaMv << -Tdelta, 0, 0, 0, 0, 0;
		GDCam.SetPose( g_dVirtPose * mvl::Cart2T( DeltaMv ) );
		GDCam.RenderToTexture( );
		GDCam.CaptureRGB( vRGB );
		_RGB2Gray( vRGB, vGDImg );
//		Error = ( vRefImg - vFDImg ).norm( );
		Error = ( vRefImg - vGDImg ).lpNorm<1>( );
		if( Error < BestError ) {
			BestMv = DeltaMv;
			BestError = Error;
		}

		// left
		DeltaMv << 0, -Tdelta, 0, 0, 0, 0;
		GDCam.SetPose( g_dVirtPose * mvl::Cart2T( DeltaMv ) );
		GDCam.RenderToTexture( );
		GDCam.CaptureRGB( vRGB );
		_RGB2Gray( vRGB, vGDImg );
//		Error = ( vRefImg - vFDImg ).norm( );
		Error = ( vRefImg - vGDImg ).lpNorm<1>( );
		if( Error < BestError ) {
			BestMv = DeltaMv;
			BestError = Error;
		}

		// right
		DeltaMv << 0, Tdelta, 0, 0, 0, 0;
		GDCam.SetPose( g_dVirtPose * mvl::Cart2T( DeltaMv ) );
		GDCam.RenderToTexture( );
		GDCam.CaptureRGB( vRGB );
		_RGB2Gray( vRGB, vGDImg );
//		Error = ( vRefImg - vFDImg ).norm( );
		Error = ( vRefImg - vGDImg ).lpNorm<1>( );
		if( Error < BestError ) {
			BestMv = DeltaMv;
			BestError = Error;
		}

		// up
		DeltaMv << 0, 0, -Tdelta, 0, 0, 0;
		GDCam.SetPose( g_dVirtPose * mvl::Cart2T( DeltaMv ) );
		GDCam.RenderToTexture( );
		GDCam.CaptureRGB( vRGB );
		_RGB2Gray( vRGB, vGDImg );
//		Error = ( vRefImg - vFDImg ).norm( );
		Error = ( vRefImg - vGDImg ).lpNorm<1>( );
		if( Error < BestError ) {
			BestMv = DeltaMv;
			BestError = Error;
		}

		// down
		DeltaMv << 0, 0, Tdelta, 0, 0, 0;
		GDCam.SetPose( g_dVirtPose * mvl::Cart2T( DeltaMv ) );
		GDCam.RenderToTexture( );
		GDCam.CaptureRGB( vRGB );
		_RGB2Gray( vRGB, vGDImg );
//		Error = ( vRefImg - vFDImg ).norm( );
		Error = ( vRefImg - vGDImg ).lpNorm<1>( );
		if( Error < BestError ) {
			BestMv = DeltaMv;
			BestError = Error;
		}

		// roll +
		DeltaMv << 0, 0, 0, Rdelta, 0, 0;
		GDCam.SetPose( g_dVirtPose * mvl::Cart2T( DeltaMv ) );
		GDCam.RenderToTexture( );
		GDCam.CaptureRGB( vRGB );
		_RGB2Gray( vRGB, vGDImg );
//		Error = ( vRefImg - vFDImg ).norm( );
		Error = ( vRefImg - vGDImg ).lpNorm<1>( );
		if( Error < BestError ) {
			BestMv = DeltaMv;
			BestError = Error;
		}

		// roll -
		DeltaMv << 0, 0, 0, -Rdelta, 0, 0;
		GDCam.SetPose( g_dVirtPose * mvl::Cart2T( DeltaMv ) );
		GDCam.RenderToTexture( );
		GDCam.CaptureRGB( vRGB );
		_RGB2Gray( vRGB, vGDImg );
//		Error = ( vRefImg - vFDImg ).norm( );
		Error = ( vRefImg - vGDImg ).lpNorm<1>( );
		if( Error < BestError ) {
			BestMv = DeltaMv;
			BestError = Error;
		}

		// pitch +
		DeltaMv << 0, 0, 0, 0, Rdelta, 0;
		GDCam.SetPose( g_dVirtPose * mvl::Cart2T( DeltaMv ) );
		GDCam.RenderToTexture( );
		GDCam.CaptureRGB( vRGB );
		_RGB2Gray( vRGB, vGDImg );
//		Error = ( vRefImg - vFDImg ).norm( );
		Error = ( vRefImg - vGDImg ).lpNorm<1>( );
		if( Error < BestError ) {
			BestMv = DeltaMv;
			BestError = Error;
		}

		// pitch -
		DeltaMv << 0, 0, 0, 0, -Rdelta, 0;
		GDCam.SetPose( g_dVirtPose * mvl::Cart2T( DeltaMv ) );
		GDCam.RenderToTexture( );
		GDCam.CaptureRGB( vRGB );
		_RGB2Gray( vRGB, vGDImg );
//		Error = ( vRefImg - vFDImg ).norm( );
		Error = ( vRefImg - vGDImg ).lpNorm<1>( );
		if( Error < BestError ) {
			BestMv = DeltaMv;
			BestError = Error;
		}

		// yaw +
		DeltaMv << 0, 0, 0, 0, 0, Rdelta;
		GDCam.SetPose( g_dVirtPose * mvl::Cart2T( DeltaMv ) );
		GDCam.RenderToTexture( );
		GDCam.CaptureRGB( vRGB );
		_RGB2Gray( vRGB, vGDImg );
//		Error = ( vRefImg - vFDImg ).norm( );
		Error = ( vRefImg - vGDImg ).lpNorm<1>( );
		if( Error < BestError ) {
			BestMv = DeltaMv;
			BestError = Error;
		}

		// yaw -
		DeltaMv << 0, 0, 0, 0, 0, -Rdelta;
		GDCam.SetPose( g_dVirtPose * mvl::Cart2T( DeltaMv ) );
		GDCam.RenderToTexture( );
		GDCam.CaptureRGB( vRGB );
		_RGB2Gray( vRGB, vGDImg );
//		Error = ( vRefImg - vFDImg ).norm( );
		Error = ( vRefImg - vGDImg ).lpNorm<1>( );
		if( Error < BestError ) {
			BestMv = DeltaMv;
			BestError = Error;
		}

		//----------------------------------

		// move camera to pose with less error
		g_dVirtPose = g_dVirtPose * mvl::Cart2T( BestMv );
		VirtCam.SetPose( g_dVirtPose );
		//std::cout << std::endl << "Best move is: " << BestMv.transpose() << std::endl;
	}
}


/////////////////////////////////////////////////////////////////////////////////////////

void ESM( )
{
	// Generators
	std::vector < Eigen::Matrix4d > Gen;
	Gen.resize( 6 );

	// 1st generator
	Gen[0] << 0, 0, 0, 1,
			0, 0, 0, 0,
			0, 0, 0, 0,
			0, 0, 0, 0;

	// 2nd generator
	Gen[1] << 0, 0, 0, 0,
			0, 0, 0, 1,
			0, 0, 0, 0,
			0, 0, 0, 0;

	// 3rd generator
	Gen[2] << 0, 0, 0, 0,
			0, 0, 0, 0,
			0, 0, 0, 1,
			0, 0, 0, 0;

	// 4th generator
	Gen[3] << 0, 0, 0, 0,
			0, 0, -1, 0,
			0, 1, 0, 0,
			0, 0, 0, 0;

	// 5th generator
	Gen[4] << 0, 0, 1, 0,
			0, 0, 0, 0,
			-1, 0, 0, 0,
			0, 0, 0, 0;

	// 6th generator
	Gen[5] << 0, -1, 0, 0,
			1, 0, 0, 0,
			0, 0, 0, 0,
			0, 0, 0, 0;

	while( 1 ) {
		if( g_DoESM ) {

			Eigen::Matrix4d dInitialVirtPose = g_dVirtPose;

			// this is the variable we update and at the end
			// will hold the final transformation between the two cameras
			// initially it is set to the identity
			Eigen::Matrix4d dTrv = Eigen::Matrix4d::Identity( );

			// image holders
			std::vector < unsigned char> vRGB; // original RGB image
			Eigen::VectorXf vRefImg; // grayscale image
			Eigen::VectorXf vVirtImg; // grayscale image
			Eigen::VectorXf vVirtDepth; // depth image

			// resize vectors
			vRGB.resize( g_nImgWidth * g_nImgHeight * 3 );
			vRefImg.resize( g_nImgWidth * g_nImgHeight );
			vVirtImg.resize( g_nImgWidth * g_nImgHeight );
			vVirtDepth.resize( g_nImgWidth * g_nImgHeight );

			// populate vectors & convert to grayscale
			RefCam.CaptureRGB( vRGB );
			_RGB2Gray( vRGB, vRefImg );
			VirtCam.CaptureRGB( vRGB );
			_RGB2Gray( vRGB, vVirtImg );
			VirtCam.CaptureDepth( vVirtDepth.data( ) );
			_FlipDepth( vVirtDepth );

			/*
			std::cout << "Blah: " << std::endl;
			for( int ii = 0; ii < 12; ii++ ) {
				for( int jj = 0; jj < 12; jj++ ) {
					std::cout << vRefImg[ii*12+jj] << " ";
				}
				std::cout << std::endl;
			}
			std::cout << std::endl;
			std::cout << "Blah: " << std::endl;
			for( int ii = 0; ii < 12; ii++ ) {
				for( int jj = 0; jj < 12; jj++ ) {
					std::cout << vVirtImg[ii*12+jj] << " ";
				}
				std::cout << std::endl;
			}
			std::cout << std::endl;
			std::cout << "Blah: " << std::endl;
			for( int ii = 0; ii < 12; ii++ ) {
				for( int jj = 0; jj < 12; jj++ ) {
					std::cout << vVirtDepth[ii*12+jj] << " ";
				}
				std::cout << std::endl;
			}
			std::cout << std::endl;
			exit(0);
			*/

			// image error
			Eigen::VectorXf ImgError;
			ImgError = vRefImg - vVirtImg;

			// print initial poses
			std::cout << "Reference Pose: " << mvl::T2Cart( g_dRefPose ).transpose( ) << std::endl;
			std::cout << "Initial Virtual Pose: " << mvl::T2Cart( g_dVirtPose ).transpose( ) << std::endl;
			std::cout << "Error is: " << ImgError.lpNorm<1>( ) / (128*128)  << std::endl;

			// hard limit of iterations so we don't loop forever
			int nMaxIters = 0;

			while( /* ImgError.norm( ) */ 200 > 100.0 && nMaxIters < 10 && g_DoESM ) {

				// increment counter
				nMaxIters++;

				// Jacobian
				Eigen::Matrix<double, 1, 6 > J;
				J.setZero();

				// LHS
				Eigen::Matrix<double, 6, 6 > LHS;
				LHS.setZero();

				// RHS
				Eigen::Matrix<double, 6, 1 > RHS;
				RHS.setZero();

				double error = 0;
				unsigned int errorPts = 0;


				for( int ii = 0; ii < g_nImgHeight; ii++ ) {
					for( int jj = 0; jj < g_nImgWidth; jj++ ) {

						// variables
						Eigen::Vector2d Ur; // pixel position
						Eigen::Vector3d Pr, Pv; // 3d point
						Eigen::Vector4d Ph; // homogenized point

						// check if pixel is contained in our model (i.e. has depth)
						if( vVirtDepth[ii * g_nImgWidth + jj] == 0 ) {
							continue;
						}


						//--------------------- first term 1x2

						// evaluate 'a' = L[ Trv * Linv( Uv ) ]
						// back project to virtual camera's reference frame
						// this already brings points to robotics reference frame
						Pv = _BackProject( jj, ii, vVirtDepth[ii * g_nImgWidth + jj] );

						// convert to homogeneous coordinate
						Ph << Pv, 1;

						// transform point to reference camera's frame
						// Pr = Trv * Pv
						Ph = dTrv * Ph;
						Pr = Ph.head(3);

						// project onto reference camera
						Eigen::Vector3d Lr;
						Lr = _Project( Pr );
						Ur = Lr.head(2);
						Ur = Ur / Lr(2);
						// these 2 should be the same
						//Ur = mvl::Project3dTo2dRoboticsFrame( RefCam.GetKMatrix( ), dTrv, Pv );
						//Ur = mvl::Project3dTo2dRoboticsFrame( RefCam.GetKMatrix( ), Eigen::Matrix4d::Identity( ), Pr );


						// check if point falls in camera's field of view
						if( Ur( 0 ) <= 1 || Ur( 0 ) >= g_nImgWidth-2 || Ur( 1 ) <= 1 || Ur( 1 ) >= g_nImgHeight-2 ) {
							continue;
						}

						// finite differences
						float TopPix = _Interpolate( Ur( 0 ), Ur( 1 ) - 1, vRefImg );
						float BotPix = _Interpolate( Ur( 0 ), Ur( 1 ) + 1, vRefImg );
						float LeftPix = _Interpolate( Ur( 0 ) - 1, Ur( 1 ), vRefImg );
						float RightPix = _Interpolate( Ur( 0 ) + 1, Ur( 1 ), vRefImg );

						Eigen::Matrix<double, 1, 2 > Term1;
						Term1( 0 ) = ( RightPix - LeftPix ) / 2.0;
						Term1( 1 ) = ( BotPix - TopPix ) / 2.0;


						//--------------------- second term 2x3

						// evaluate 'b' = Trv * Linv( Uv )
						// this was already calculated for Term1

						// fill matrix
						// 1/c      0       -a/c^2
						//  0       1/c     -b/c^2
						Eigen::Matrix< double, 2, 3 > Term2;
						double PowC = Lr(2) * Lr(2);
						Term2( 0, 0 ) = 1.0 / Lr(2);
						Term2( 0, 1 ) = 0;
						Term2( 0, 2 ) = -(Lr( 0 )) / PowC;
						Term2( 1, 0 ) = 0;
						Term2( 1, 1 ) = 1.0 / Lr(2);
						Term2( 1, 2 ) = -(Lr( 1 )) / PowC;
						// permutation matrix
						Eigen::Matrix3d M;
						M << 0, 1, 0,
							 0, 0, 1,
							 1, 0, 0;
						Term2 = Term2 * RefCam.GetKMatrix( ) * M;


						/*
						// Finite Differences
						Eigen::Matrix< double, 2, 3 > Term2_FD;
						double eps = 1e-4;
						Eigen::Vector3d Pt;
						Pt = Pv;
						Pt(0) += eps;
						Eigen::Vector2d Ut;
						Ut = mvl::Project3dTo2dRoboticsFrame( RefCam.GetKMatrix( ), dTrv, Pt );
						Ut = Ut - Ur;
						Term2_FD(0, 0) = Ut(0);
						Term2_FD(1, 0) = Ut(1);

						Pt = Pv;
						Pt(1) += eps;
						Ut = mvl::Project3dTo2dRoboticsFrame( RefCam.GetKMatrix( ), dTrv, Pt );
						Ut = Ut - Ur;
						Term2_FD(0,1) = Ut(0);
						Term2_FD(1,1) = Ut(1);

						Pt = Pv;
						Pt(2) += eps;
						Ut = mvl::Project3dTo2dRoboticsFrame( RefCam.GetKMatrix( ), dTrv, Pt );
						Ut = Ut - Ur;
						Term2_FD(0,2) = Ut(0);
						Term2_FD(1,2) = Ut(1);
						Term2_FD = Term2_FD / eps;
						/* */


						//--------------------- third term 3x1

						// we need Pv in homogenous coordinates
						Ph << Pv, 1;

						Eigen::Vector4d Term3i;
						// last row of Term3 is truncated since it is always 0
						Eigen::Vector3d Term3;

						// fill Jacobian with T generators

						Term3i = dTrv * Gen[0] * Ph;
						Term3 = Term3i.head( 3 );
						J( 0, 0 ) = Term1 * Term2 * Term3;

						Term3i = dTrv * Gen[1] * Ph;
						Term3 = Term3i.head( 3 );
						J( 0, 1 ) = Term1 * Term2 * Term3;

						Term3i = dTrv * Gen[2] * Ph;
						Term3 = Term3i.head( 3 );
						J( 0, 2 ) = Term1 * Term2 * Term3;

						Term3i = dTrv * Gen[3] * Ph;
						Term3 = Term3i.head( 3 );
						J( 0, 3 ) = Term1 * Term2 * Term3;

						Term3i = dTrv * Gen[4] * Ph;
						Term3 = Term3i.head( 3 );
						J( 0, 4 ) = Term1 * Term2 * Term3;

						Term3i = dTrv * Gen[5] * Ph;
						Term3 = Term3i.head( 3 );
						J( 0, 5 ) = Term1 * Term2 * Term3;



						/*
						// Finite Differences of whole Jacobian
						double eps = 1e-4;
						Eigen::Vector6d J_FD;
						Eigen::Matrix4d T;
						Eigen::Vector6d Tc;
						Sophus::SE3 D;


						Tc.setZero();
						Tc(0) = eps;
						Ph << Pv, 1;
						D = Sophus::SE3::exp( Tc );
						Ph = D.matrix() * Ph;
						Pr = Ph.head(3);
						Lr = _Project( Pr );
						Ur = Lr.head(2);
						Ur = Ur / Lr(2);
						// check if point falls in camera's field of view
						if( Ur( 0 ) <= 1 || Ur( 0 ) >= g_nImgWidth-2 || Ur( 1 ) <= 1 || Ur( 1 ) >= g_nImgHeight-2 ) {
							continue;
						}
						J_FD(0) = _Interpolate( Ur( 0 ), Ur( 1 ), vRefImg ) - vVirtImg[ii * g_nImgWidth + jj];
						J_FD(0) -= vRefImg[ii * g_nImgWidth + jj] - vVirtImg[ii * g_nImgWidth + jj];

						Tc.setZero();
						Tc(1) = eps;
						Ph << Pv, 1;
						D = Sophus::SE3::exp( Tc );
						Ph = D.matrix() * Ph;
						Pr = Ph.head(3);
						Lr = _Project( Pr );
						Ur = Lr.head(2);
						Ur = Ur / Lr(2);
						// check if point falls in camera's field of view
						if( Ur( 0 ) <= 1 || Ur( 0 ) >= g_nImgWidth-2 || Ur( 1 ) <= 1 || Ur( 1 ) >= g_nImgHeight-2 ) {
							continue;
						}
						J_FD(1) = _Interpolate( Ur( 0 ), Ur( 1 ), vRefImg ) - vVirtImg[ii * g_nImgWidth + jj];
						J_FD(1) -= vRefImg[ii * g_nImgWidth + jj] - vVirtImg[ii * g_nImgWidth + jj];

						Tc.setZero();
						Tc(2) = eps;
						Ph << Pv, 1;
						D = Sophus::SE3::exp( Tc );
						Ph = D.matrix() * Ph;
						Pr = Ph.head(3);
						Lr = _Project( Pr );
						Ur = Lr.head(2);
						Ur = Ur / Lr(2);
						// check if point falls in camera's field of view
						if( Ur( 0 ) <= 1 || Ur( 0 ) >= g_nImgWidth-2 || Ur( 1 ) <= 1 || Ur( 1 ) >= g_nImgHeight-2 ) {
							continue;
						}
						J_FD(2) = _Interpolate( Ur( 0 ), Ur( 1 ), vRefImg ) - vVirtImg[ii * g_nImgWidth + jj];
						J_FD(2) -= vRefImg[ii * g_nImgWidth + jj] - vVirtImg[ii * g_nImgWidth + jj];

						Tc.setZero();
						Tc(3) = eps;
						Ph << Pv, 1;
						D = Sophus::SE3::exp( Tc );
						Ph = D.matrix() * Ph;
						Pr = Ph.head(3);
						Lr = _Project( Pr );
						Ur = Lr.head(2);
						Ur = Ur / Lr(2);
						// check if point falls in camera's field of view
						if( Ur( 0 ) <= 1 || Ur( 0 ) >= g_nImgWidth-2 || Ur( 1 ) <= 1 || Ur( 1 ) >= g_nImgHeight-2 ) {
							continue;
						}
						J_FD(3) = _Interpolate( Ur( 0 ), Ur( 1 ), vRefImg ) - vVirtImg[ii * g_nImgWidth + jj];
						J_FD(3) -= vRefImg[ii * g_nImgWidth + jj] - vVirtImg[ii * g_nImgWidth + jj];

						Tc.setZero();
						Tc(4) = eps;
						Ph << Pv, 1;
						D = Sophus::SE3::exp( Tc );
						Ph = D.matrix() * Ph;
						Pr = Ph.head(3);
						Lr = _Project( Pr );
						Ur = Lr.head(2);
						Ur = Ur / Lr(2);
						// check if point falls in camera's field of view
						if( Ur( 0 ) <= 1 || Ur( 0 ) >= g_nImgWidth-2 || Ur( 1 ) <= 1 || Ur( 1 ) >= g_nImgHeight-2 ) {
							continue;
						}
						J_FD(4) = _Interpolate( Ur( 0 ), Ur( 1 ), vRefImg ) - vVirtImg[ii * g_nImgWidth + jj];
						J_FD(4) -= vRefImg[ii * g_nImgWidth + jj] - vVirtImg[ii * g_nImgWidth + jj];

						Tc.setZero();
						Tc(5) = eps;
						Ph << Pv, 1;
						D = Sophus::SE3::exp( Tc );
						Ph = D.matrix() * Ph;
						Pr = Ph.head(3);
						Lr = _Project( Pr );
						Ur = Lr.head(2);
						Ur = Ur / Lr(2);
						// check if point falls in camera's field of view
						if( Ur( 0 ) <= 1 || Ur( 0 ) >= g_nImgWidth-2 || Ur( 1 ) <= 1 || Ur( 1 ) >= g_nImgHeight-2 ) {
							continue;
						}
						J_FD(5) = _Interpolate( Ur( 0 ), Ur( 1 ), vRefImg ) - vVirtImg[ii * g_nImgWidth + jj];
						J_FD(5) -= vRefImg[ii * g_nImgWidth + jj] - vVirtImg[ii * g_nImgWidth + jj];

						J_FD = J_FD / eps;

						std::cout << "J: " << J << std::endl;
						std::cout << "J_FD: " << J_FD.transpose() << std::endl;
						/**/


						// estimate LHS (Hessian)
						// LHS = Hessian = Jt * J
						LHS += J.transpose( ) * J;
						//std::cout << "J is: " << std::endl << J << std::endl;

						// estimate RHS (error)
						// RHS = Jt * e
						error += abs( _Interpolate( Ur( 0 ), Ur( 1 ), vRefImg ) - vVirtImg[ii * g_nImgWidth + jj]);
						errorPts++;
						RHS += J.transpose( ) * ( _Interpolate( Ur( 0 ), Ur( 1 ), vRefImg ) - vVirtImg[ii * g_nImgWidth + jj] );
					}
				}
				std::cout << "LHS is: " << std::endl << LHS << std::endl;
				std::cout << "RHS is: " << RHS.transpose( ) << std::endl;

				// calculate deltaPose as Hinv * Jt * error
				//deltaPose = LHS.inverse() * RHS;
				Eigen::Vector6d Delta;
				Delta = LHS.ldlt( ).solve( RHS );

				// convert Delta from Lie
				Sophus::SE3 DeltaPose;
				DeltaPose = Sophus::SE3::exp( Delta );

				std::cout << "Delta Pose is: " << mvl::T2Cart( DeltaPose.matrix( ) ).transpose( ) << std::endl;

				// update Trv
				dTrv = dTrv * mvl::TInv( DeltaPose.matrix( ) );

				// update virtual camera position
				// Tr = Tv * Tvr = Tv * I * Delta1 * Delta2 * etc
				// REMEMBER to convert from vision frame to robotics frame

				// dTrv is the delta from Reference camera to Virtual camera
				// This pose in global reference frame is:
				// Twv = Twr * Trv
				g_dVirtPose = dInitialVirtPose * mvl::TInv( dTrv );
//				g_dVirtPose = dInitialVirtPose * dTrv;
				VirtCam.SetPose( g_dVirtPose );
				std::cout << "New Virtual Pose is: " << mvl::T2Cart( g_dVirtPose ).transpose( ) << std::endl;

				// we cannot RenderToTexture() here since it is a different thread
				// we need to synchronize with the GUI so that the Virtual Camera is
				// rendered at the new pose we just updated.
				//VirtCam.RenderToTexture( );
				//g_Mutex = true;
				//while( g_Mutex ) {
				//}

				// at this point the FB has filled with the new camera's position
				//VirtCam.CaptureRGB( vRGB );
				//_RGB2Gray( vRGB, vVirtImg );
				//VirtCam.CaptureDepth( vVirtDepth.data( ) );
				//_FlipDepth( vVirtDepth );

				// calculate new error
				//ImgError = vRefImg - vVirtImg;
				//std::cout << "Error is: " << ImgError.norm( ) << std::endl;
				std::cout << "Error is: " << error / errorPts << " given by: "<< errorPts << " points." << std::endl;
			}

			std::cout << "Reference Pose: " << mvl::T2Cart( g_dRefPose ).transpose( ) << std::endl;
			std::cout << "Final Estimated Pose: " << mvl::T2Cart( g_dVirtPose ).transpose( ) << std::endl;
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

	// init window
	GuiWindow* pWin = new GuiWindow( 0, 0, 1024, 640, "Dense Pose Refinement" );

	// load image to be used for texture
	cv::Mat Img;
	Img = cv::imread( "antoine.jpg", 0 );
	cv::transpose( Img, Img );

	// initialize image plane
	GLImgPlane glImgPlane;
	glImgPlane.SetImage( Img.data, Img.cols, Img.rows, GL_LUMINANCE, GL_UNSIGNED_BYTE );
	Eigen::Vector6d BasePose;
	BasePose << 0, -100, 100, 0, 0, 0;
	glImgPlane.SetBaseFrame( BasePose );

	// set up image difference container
	glImgDiff.InitReset( );
	glImgDiff.SetVisible( );
	glImgDiff.SetPerceptable( false );

	// register objects
	pWin->AddChildToRoot( &glImgPlane );
	pWin->AddChildToRoot( &glImgDiff );


	// prepare K matrix
	Eigen::Matrix3d dK; // computer vision K matrix
	dK << g_nImgWidth, 0, g_nImgWidth / 2,
			0, g_nImgHeight, g_nImgHeight / 2,
			0, 0, 1;

	// initialize cameras
	RefCam.Init( &pWin->SceneGraph( ), g_dRefPose, dK, g_nImgWidth, g_nImgHeight, eSimCamRGB );
	VirtCam.Init( &pWin->SceneGraph( ), g_dVirtPose, dK, g_nImgWidth, g_nImgHeight, eSimCamDepth | eSimCamRGB );
	GDCam.Init( &pWin->SceneGraph( ), g_dVirtPose, dK, g_nImgWidth, g_nImgHeight, eSimCamRGB );

	// set up lighting
	glEnable( GL_LIGHT0 ); // activate light0
	glEnable( GL_LIGHTING ); // enable lighting

	// look at a nice place
	pWin->LookAt( -50, 10, -10, 0, 10, 0, 0, 0, -1 );

	// add our callbacks
	pWin->AddPreRenderCallback( UpdateCameraPose, NULL );
	pWin->AddPostRenderCallback( DoPoseRefinement, NULL );
	pWin->AddPostRenderCallback( ShowCameraAndTextures, NULL );

	// need to launch ESM thread
	boost::thread ESM_Thread( ESM );

	return( pWin->Run( ) );
}