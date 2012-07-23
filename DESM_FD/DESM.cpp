#include <SimpleGui/Gui.h>

#include <boost/thread.hpp>

#include <Mvlpp/Mvl.h>

#include <opencv2/highgui/highgui.hpp>

#include "GLMosaic.h"
#include "GLHeightMap.h"
#include "PeaksHeightMap.h"

// Global CVars
bool& g_bShowFrustum = CVarUtils::CreateCVar( "ShowFrustum", true, "Show camera viewing frustum." );

GLMosaic glMosaic;
//	Eigen::Vector6d MosPosition = Eigen::Vector6d::Zero();
//	Eigen::Vector6d MosPos = Eigen::Vector6d::Zero();

// Global Vars
GLImage glImgDiff;
int g_nImgWidth = 128;
int g_nImgHeight = 128;
bool g_DoESM = false;
bool g_Mutex;


// Camera
GLSimCam RefCam; // reference camera we move
GLSimCam VirtCam; // virtual camera which calculates transformation

// Camera Poses
Eigen::Matrix4d g_dRefPose = mvl::Cart2T( -30, -3, 0, 0, 0, 0 ); // initial camera pose
Eigen::Matrix4d g_dVirtPose = g_dRefPose;


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
				//				MosPos(0) += 0.01;
				g_dRefVel(0) += 0.01;
				break;
			case 'q': case 'Q':
				//				MosPos(0) -= 0.01;
				g_dRefVel(0) -= 0.01;
				break;
				// right
			case 'd': case 'D':
				//				MosPos(1) += 0.01;
				g_dRefVel(1) += 0.01;
				break;
			case 'a': case 'A':
				//				MosPos(1) -= 0.01;
				g_dRefVel(1) -= 0.01;
				break;
				// down
			case 's': case 'S':
				//				MosPos(2) += 0.01;
				g_dRefVel(2) += 0.01;
				break;
			case 'w': case 'W':
				//				MosPos(2) -= 0.01;
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
				//				MosPos << 0,0,0,0,0,0;
				g_dRefVel << 0,0,0,0,0,0;
				break;
			case 't': case 'T':
				g_DoESM = true;
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

Eigen::Vector4d _Robotic2Vision( Eigen::Vector4d P )
{
	Eigen::Vector4d T;
	T( 0 ) = P( 1 );
	T( 1 ) = P( 2 );
	T( 2 ) = P( 0 );
	T( 3 ) = 1;
	return T;
}


/////////////////////////////////////////////////////////////////////////////////////////

Eigen::Matrix4d _Robotic2Vision( Eigen::Matrix4d M )
{
	return mvl::Aero2VisionInplace( M );
}


/////////////////////////////////////////////////////////////////////////////////////////

Eigen::Matrix4d _Robotic2Vision( Eigen::Vector6d Pose )
{
	Eigen::Matrix4d M = mvl::Cart2T( Pose );
	return mvl::Aero2VisionInplace( M );
}


/////////////////////////////////////////////////////////////////////////////////////////

Eigen::Vector3d _Vision2Robotic( Eigen::Vector3d P )
{
	Eigen::Vector3d T;
	T( 0 ) = P( 2 );
	T( 1 ) = P( 0 );
	T( 2 ) = P( 1 );
	return T;
}


/////////////////////////////////////////////////////////////////////////////////////////

Eigen::Vector4d _Vision2Robotic( Eigen::Vector4d P )
{
	Eigen::Vector4d T;
	T( 0 ) = P( 2 );
	T( 1 ) = P( 0 );
	T( 2 ) = P( 1 );
	T( 3 ) = 1;
	return T;
}


/////////////////////////////////////////////////////////////////////////////////////////

Eigen::Matrix4d _Vision2Robotic( Eigen::Matrix4d M )
{
	return mvl::Vision2AeroInplace( M );
}


/////////////////////////////////////////////////////////////////////////////////////////

Eigen::Matrix4d _Vision2Robotic( Eigen::Vector6d Pose )
{
	Eigen::Matrix4d M = mvl::Cart2T( Pose );
	return mvl::Vision2AeroInplace( M );
}


/////////////////////////////////////////////////////////////////////////////////////////

Eigen::Vector2d _Project( const Eigen::Vector4d& P )
{
	// temporal working point
	Eigen::Vector3d T = P.block < 3, 1 > ( 0, 0 );

	// get camera intrinsics
	Eigen::Matrix3d K = RefCam.GetKMatrix( );
	T = K * T;
	if( T( 2 ) == 0 ) {
		std::cout << "CRAP! " << T.transpose( ) << std::endl;
	}

	// de-homogenize
	T = T / T( 2 );

	return T.block < 2, 1 > ( 0, 0 );
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
	// set into robotics coordinate frame
	P( 0 ) = Depth * ( ( X - cx ) / fx );
	P( 1 ) = Depth * ( ( Y - cy ) / fy );
	P( 2 ) = Depth;

	// convert to robotic frame
	P = _Vision2Robotic( P );

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
	//	MosPosition += MosPos;
	//	glMosaic.SetBaseFrame( mvl::Cart2T( MosPosition ));

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

	// calculate difference & normalize
	vErrorImg = ( vRefImg - vVirtImg ) / 255;

	/*
	// Point in in world (robotic) reference frame
	Eigen::Vector2d U;
	U = mvl::Project3dTo2dRoboticsFrame( RefCam.GetKMatrix(), g_dRefPose, MosPosition.head(3));
	std::cout << "U: " << U.transpose() << std::endl;

	unsigned int idx = (int(U(1)) * g_nImgWidth) + int(U(0));
	vErrorImg = vRefImg - vRefImg;
	vErrorImg[ idx ] = 1.0;

	Eigen::VectorXf vVirtDepth; // grayscale image
	vVirtDepth.resize( g_nImgWidth * g_nImgHeight );
	VirtCam.CaptureDepth( vVirtDepth.data( ) );

	for( int ii = 0; ii < vVirtDepth.size(); ii++ ) {
		if( vVirtDepth[ii] != 0 ) {
			std::cout << "Found something at " << ii << " " << vVirtDepth[ii] << std::endl;
		}
	}

	Eigen::Vector3d P3;
	P3 = _BackProject( U(0), U(1), vVirtDepth[idx] );
	std::cout << "P3: " << P3.transpose() << std::endl;
	P3 = _Vision2Robotic(P3);
	std::cout << "P3: " << P3.transpose() << std::endl;
	 */

	glImgDiff.SetImage( (unsigned char*) vErrorImg.data( ), g_nImgWidth, g_nImgHeight, GL_LUMINANCE, GL_FLOAT );
	glImgDiff.SetSizeAsPercentageOfWindow( 0.66, 0.66, 1, 1 );
	DrawBorderAsWindowPercentage( 0.66, 0.66, 1, 1 );
//	printf( "\rNormalized Image Error: %f", vErrorImg.norm( ) );
//	fflush( stdout );
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
				J = Eigen::Matrix< double, 1, 6 > ::Zero( );

				// LHS
				Eigen::Matrix<double, 6, 6 > LHS;
				LHS = Eigen::Matrix< double, 6, 6 > ::Zero( );

				// RHS
				Eigen::Matrix<double, 6, 1 > RHS;
				RHS = Eigen::Matrix< double, 6, 1 > ::Zero( );

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

						// transform point to real camera's reference frame
						// Trv * P
						Ph = dTrv * Ph;
						Pr = Ph.head(3);

						// project into reference camera's frame
						//Ur = _Project( Ph );
						Ur = mvl::Project3dTo2dRoboticsFrame( RefCam.GetKMatrix( ), Eigen::Matrix4d::Identity( ), Pr );

						// check if point falls in camera's field of view
						if( Ur( 0 ) < 0 || Ur( 0 ) >= g_nImgWidth || Ur( 1 ) < 0 || Ur( 1 ) >= g_nImgHeight ) {
							continue;
						}

						// finite differences
						float TopPix = _Interpolate( Ur( 0 ), Ur( 1 ) - 1, vRefImg );
						float BotPix = _Interpolate( Ur( 0 ), Ur( 1 ) + 1, vRefImg );
						float LeftPix = _Interpolate( Ur( 0 ) - 1, Ur( 1 ), vRefImg );
						float RightPix = _Interpolate( Ur( 0 ) + 1, Ur( 1 ), vRefImg );

						Eigen::Matrix<double, 1, 2 > Term1;
						Term1( 0 ) = ( LeftPix - RightPix ) / 2.0;
						Term1( 1 ) = ( TopPix - BotPix ) / 2.0;


						//--------------------- second term 2x3

						// evaluate 'b' = Trv * Linv( Uv )
						// this was already calculated for Term1

						// fill matrix
						// 1/c      0       -a/c^2
						//  0       1/c     -b/c^2
						Eigen::Matrix< double, 2, 3 > Term2;
						// the projection function already de-homogenizes therefore c = 1
						Term2( 0, 0 ) = 1;
						Term2( 0, 1 ) = 0;
						Term2( 0, 2 ) = -(Ur( 0 ));
						Term2( 1, 0 ) = 0;
						Term2( 1, 1 ) = 1;
						Term2( 1, 2 ) = -(Ur( 1 ));


						//--------------------- third term 3x1

						// we need Pv in homogenous coordinates
						Ph << Pv, 1;

						Eigen::Vector4d Term3i;
						// last row of Term3 is truncated since it is always 0
						Eigen::Vector3d Term3;

						// fill Jacobian with T generators

						Term3i = dTrv * Gen[0] * Ph;
						//Term3i = Gen[0] * Ph;
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
				//Sophus::SE3 DeltaPose;
				//DeltaPose = Sophus::SE3::exp( Delta );

				std::cout << "Delta Pose is: " << mvl::T2Cart( DeltaPose.matrix( ) ).transpose( ) << std::endl;

				// update Trv
				dTrv = dTrv * DeltaPose.matrix( );

				// update virtual camera position
				// Tr = Tv * Tvr = Tv * I * Delta1 * Delta2 * etc
				// REMEMBER to convert from vision frame to robotics frame

				// dTrv is the delta from Reference camera to Virtual camera
				// This pose in global reference frame is:
				// Twv = Twr * Trv
				g_dVirtPose = dInitialVirtPose * mvl::TInv( dTrv );
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
				std::cout << "Error is: " << error / errorPts << " give by: "<< errorPts << " points." << std::endl;
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
	std::string sMesh = cl.follow( "antoine.obj", "-mesh" );

	// init window
	GuiWindow* pWin = new GuiWindow( 0, 0, 1024, 640, "Dense Pose Refinement" );

	GLGrid glGrid;
	glGrid.SetPerceptable( false );

	//	GLMesh glMesh;
	//	glMesh.Init( sMesh );

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
	pWin->AddChildToRoot( &glGrid );
	pWin->AddChildToRoot( &glImgDiff );

	//	glMesh.SetPerceptable( true );
	//	glMesh.SetVisible();
	//	glMesh.SetPose( 0, 0, -1, 0, 0, 0 );

	Eigen::Matrix3d dK; // computer vision K matrix
	dK << g_nImgWidth, 0, g_nImgWidth / 2,
			0, g_nImgHeight, g_nImgHeight / 2,
			0, 0, 1;

	RefCam.Init( &pWin->SceneGraph( ), g_dRefPose, dK, g_nImgWidth, g_nImgHeight, eSimCamRGB );
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
