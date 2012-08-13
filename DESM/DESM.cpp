
#include <SimpleGui/Gui.h>
#include <boost/thread.hpp>
#include <Mvlpp/Mvl.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include "GLImgPlane.h"
#include "CVarHelpers.h"
#include "LinearSystem.h"

/**************************************************************************************************
 *
 * VARIABLES
 *
 **************************************************************************************************/

#define IMG_HEIGHT 512
#define IMG_WIDTH  512

// GL Objects
GLImage    glImgDiff;
GLImgPlane glImgPlane;

// Global CVars
bool&            g_bShowFrustum   = CVarUtils::CreateCVar( "Cam.ShowFrustum", true, "Show cameras viewing frustum." );
unsigned int&    g_nMaxIterations = CVarUtils::CreateCVar( "ESM.MaxIterations", 500u, "Max number of iterations." );
Eigen::Vector6d& g_dRefPose       = CVarUtils::CreateCVar( "Cam.Pose.Ref", Eigen::Vector6d( Eigen::Vector6d::Zero() ),
                                  "Reference camera's pose." );
Eigen::Vector6d& g_dVirtPose = CVarUtils::CreateCVar( "Cam.Pose.Virt", Eigen::Vector6d( Eigen::Vector6d::Zero() ),
                                   "Virtual camera's pose." );

// Global Vars
volatile bool g_bLocalize = false;
volatile bool g_bRendered = false;

// Cameras
GLSimCam RefCam;     // reference camera
GLSimCam VirtCam;    // virtual camera from which calculates transformation

// Reference Camera Controls
Eigen::Vector6d g_dRefVel = Eigen::Vector6d::Zero();

// ///////////////////////////////////////////////////////////////////////////////////////
class GuiWindow:
    public GLWindow
{
    public:
        GuiWindow(
                int         x,
                int         y,
                int         w,
                int         h,
                const char* l = 0
                ):
            GLWindow(
                x,
                y,
                w,
                h,
                l
                )
        {}

        virtual int handle(
                int e
                )
         {
            if( (e == FL_KEYBOARD) &&!m_Console.IsOpen() ) {
                switch( Fl::event_key() ) {
                    // forward
                    case 'e':
                    case 'E':
                        g_dRefVel( 0 ) += 0.01;
                        break;
                    case 'q':
                    case 'Q':
                        g_dRefVel( 0 ) -= 0.01;
                        break;

                    // right
                    case 'd':
                    case 'D':
                        g_dRefVel( 1 ) += 0.01;
                        break;
                    case 'a':
                    case 'A':
                        g_dRefVel( 1 ) -= 0.01;
                        break;

                    // down
                    case 's':
                    case 'S':
                        g_dRefVel( 2 ) += 0.01;
                        break;
                    case 'w':
                    case 'W':
                        g_dRefVel( 2 ) -= 0.01;
                        break;

                    // pitch
                    case 'i':
                    case 'I':
                        g_dRefVel( 4 ) += 0.005;
                        break;
                    case 'k':
                    case 'K':
                        g_dRefVel( 4 ) -= 0.005;
                        break;

                    // yaw
                    case 'l':
                    case 'L':
                        g_dRefVel( 5 ) += 0.005;
                        break;
                    case 'j':
                    case 'J':
                        g_dRefVel( 5 ) -= 0.005;
                        break;

                    // roll
                    case 'u':
                    case 'U':
                        g_dRefVel( 3 ) -= 0.005;
                        break;
                    case 'o':
                    case 'O':
                        g_dRefVel( 3 ) += 0.005;
                        break;
                    case ' ':
                        g_dRefVel << 0, 0, 0, 0, 0, 0;
                        break;
                    case 't':
                    case 'T':
                        g_bLocalize = true;
                        break;
                    case 'r':
                    case 'R':
                        g_bLocalize     = false;
                        g_dVirtPose = g_dRefPose;

                        VirtCam.SetPose( mvl::Cart2T( g_dVirtPose ) );
                        break;
                }
            }

            return SimpleDefaultEventHandler( e );
        }
};

// //////////////////////////////////////////////////////////////////////////////
void _FlipImg(
        Eigen::Matrix< unsigned char, 1, Eigen::Dynamic >& vImg    // < Input/Output: Img buffer
        )
 {
    Eigen::Matrix< unsigned char, 1, Eigen::Dynamic > tmp;

    tmp = vImg;

    unsigned int nImgHeight = RefCam.ImageHeight();
    unsigned int nImgWidth  = RefCam.ImageWidth();
    unsigned int Idx;

    for( int ii = 0; ii < nImgHeight; ii++ ) {
        for( int jj = 0; jj < nImgWidth; jj++ ) {
            Idx                           = (nImgHeight - ii - 1) * nImgWidth + jj;
            vImg[ii * nImgWidth + jj] = tmp[Idx];
        }
    }
}

// ///////////////////////////////////////////////////////////////////////////////////////
void UpdateCameraPose(
        GLWindow*,
        void*
        )
 {
    g_dRefPose = g_dRefPose + g_dRefVel;

    RefCam.SetPose( mvl::Cart2T( g_dRefPose ) );
//    glEnable( GL_LIGHTING );
//    glEnable( GL_LIGHT0 );
//    glClearColor( 0.0, 0.0, 0.0, 1 );
    RefCam.RenderToTexture();    // will render to texture, then copy texture to CPU memory
    VirtCam.RenderToTexture();
	g_bRendered = true;
}

// ///////////////////////////////////////////////////////////////////////////////////////
void ShowCameraAndTextures(
        GLWindow*,
        void*
        )
 {
    if( g_bShowFrustum ) {
        // show the camera
        RefCam.DrawCamera();
        VirtCam.DrawCamera();
    }

    // / show textures
    if( RefCam.HasGrey() ) {
        DrawTextureAsWindowPercentage( RefCam.GreyTexture(), RefCam.ImageWidth(), RefCam.ImageHeight(), 0, 0.66, 0.33,
                                       1 );
        DrawBorderAsWindowPercentage( 0, 0.66, 0.33, 1 );
    }

    if( VirtCam.HasGrey() ) {
        DrawTextureAsWindowPercentage( VirtCam.GreyTexture(), VirtCam.ImageWidth(), VirtCam.ImageHeight(), 0.33, 0.66,
                                       0.66, 1 );
        DrawBorderAsWindowPercentage( 0.33, 0.66, 0.66, 1 );
    }

    // draw difference image
    Eigen::Matrix< unsigned char, 1, Eigen::Dynamic > vRefImg;
    Eigen::Matrix< unsigned char, 1, Eigen::Dynamic > vVirtImg;
    Eigen::Matrix< unsigned char, 1, Eigen::Dynamic > vErrorImg;
    unsigned int               nImgWidth  = RefCam.ImageWidth();
    unsigned int               nImgHeight = RefCam.ImageHeight();

    // resize vectors
    vRefImg.resize( nImgWidth * nImgHeight );
    vVirtImg.resize( nImgWidth * nImgHeight );

    // populate vectors
	RefCam.CaptureGrey( vRefImg.data() );
	VirtCam.CaptureGrey( vVirtImg.data() );

    // calculate error
    vErrorImg = vRefImg - vVirtImg;
	_FlipImg(vErrorImg);

    glImgDiff.SetImage( (unsigned char*)vErrorImg.data(), nImgWidth, nImgHeight, GL_LUMINANCE, GL_UNSIGNED_BYTE );
    glImgDiff.SetSizeAsPercentageOfWindow( 0.66, 0.66, 1, 1 );
    DrawBorderAsWindowPercentage( 0.66, 0.66, 1, 1 );
}

// ///////////////////////////////////////////////////////////////////////////////////////
void Localizer()
 {
    LinearSystem ESM;

    while( 1 ) {
        if( g_bLocalize ) {
            Eigen::Vector6d dInitialVirtPose = g_dVirtPose;

            // print initial poses
            std::cout << "Reference Pose: " << g_dRefPose.transpose() << std::endl;
            std::cout << "Initial Virtual Pose: " << g_dVirtPose.transpose() << std::endl;

			//
			//------------------------------------- Coarse Localization
			//

			// capture reference image
			Eigen::Matrix<unsigned char, 1, Eigen::Dynamic> RefImg;
			Eigen::Matrix<unsigned char, 1, Eigen::Dynamic> RefImgD;
			RefImg.resize( IMG_HEIGHT * IMG_WIDTH );
		    RefCam.CaptureGrey( RefImg.data() );
			_FlipImg( RefImg );
			RefImgD.resize( IMG_HEIGHT/4 * IMG_WIDTH/4 );
			cv::Mat In( IMG_HEIGHT, IMG_WIDTH, CV_8UC1, RefImg.data() );
			cv::Mat Out;
			cv::resize( In, Out, cv::Size(0,0), 0.25, 0.25 );
			memcpy( RefImgD.data(), Out.data, IMG_HEIGHT/4 * IMG_WIDTH/4 );

            // initialize system of equations
            ESM.Init( RefImgD, &VirtCam, true );

            // hard limit of iterations so we don't loop forever
            int nMaxIters = 0;

            // this variable holds the estimated transform
            Eigen::Matrix4d dTrv = Eigen::Matrix4d::Identity();

            // this variable holds the delta update solution
            Eigen::Matrix4d dTdelta;

            // keep track of errors
			double NewError;
            double PrevError = ESM.Error();

			// keep track of time
			double dTs = mvl::Tic();

            while( (nMaxIters < g_nMaxIterations) && g_bLocalize ) {
                std::cout << "////////////////////////////////////////////////////////////////////////////////"
                          << std::endl;

                // increment counter
                nMaxIters++;

                // solve system
				double dTi = mvl::Tic();
                dTdelta = ESM.Solve();
				std::cout << "Solving took: " << mvl::Toc(dTi) << std::endl;

                // show solution
                std::cout << "Delta Pose is: " << mvl::T2Cart( dTdelta ).transpose() << std::endl;

                // update Trv
                ESM.ApplyUpdate();

                dTrv = dTrv * mvl::TInv( dTdelta );

                // update camera position
                g_dVirtPose = dInitialVirtPose - mvl::T2Cart( dTrv );
                VirtCam.SetPose( mvl::Cart2T( g_dVirtPose ) );

				// get error
				NewError = ESM.Error();

                std::cout << "New Virtual Pose is: " << g_dVirtPose.transpose() << std::endl;
                std::cout << "Error is: " << NewError << std::endl;

                // if error change is too small, break
                if( fabs( PrevError - NewError ) < 1e-4 ) {
                    break;
                }

				PrevError = NewError;
            }

            std::cout << std::endl << "Reference Pose: " << g_dRefPose.transpose() << std::endl;
            std::cout << "Final Estimated Pose: " << g_dVirtPose.transpose() << std::endl;
			std::cout << "Pose Difference: " << (g_dRefPose - g_dVirtPose).transpose() << std::endl;
			while(1) {}

			//
			//------------------------------------- Refine Localization
			//

			std::cout << std::endl;
			std::cout << "======================== REFINING ========================" << std::endl;
			std::cout << std::endl;

			// wait for GUI loop to render in new position
			g_bRendered = false;
			while( g_bRendered == false ) { }

			// capture reference image
		    RefCam.CaptureGrey( RefImg.data() );
			_FlipImg( RefImg );

            // initialize system of equations
            ESM.Init( RefImg, &VirtCam );

            // hard limit of iterations so we don't loop forever
            nMaxIters = 0;

            // this variable holds the estimated transform
            dTrv = Eigen::Matrix4d::Identity();

            // keep track of errors
            PrevError = ESM.Error();

			// reset initial pose
			dInitialVirtPose = g_dVirtPose;

            while( (nMaxIters < g_nMaxIterations) && g_bLocalize ) {
                std::cout << "////////////////////////////////////////////////////////////////////////////////"
                          << std::endl;

                // increment counter
                nMaxIters++;

                // solve system
				double dTi = mvl::Tic();
                dTdelta = ESM.Solve();
				std::cout << "Solving took: " << mvl::Toc(dTi) << std::endl;

                // show solution
                std::cout << "Delta Pose is: " << mvl::T2Cart( dTdelta ).transpose() << std::endl;

                // update Trv
                ESM.ApplyUpdate();

                dTrv = dTrv * mvl::TInv( dTdelta );

                // update camera position
                g_dVirtPose = dInitialVirtPose - mvl::T2Cart( dTrv );
                VirtCam.SetPose( mvl::Cart2T( g_dVirtPose ) );

				// get error
				NewError = ESM.Error();

                std::cout << "New Virtual Pose is: " << g_dVirtPose.transpose() << std::endl;
                std::cout << "Error is: " << NewError << std::endl;

                // if error change is too small, break
                if( fabs( PrevError - NewError ) < 1e-2 ) {
                    break;
                }

				PrevError = NewError;
            }
			dTs = mvl::TocMS(dTs);


            std::cout << std::endl << "Reference Pose: " << g_dRefPose.transpose() << std::endl;
            std::cout << "Final Estimated Pose: " << g_dVirtPose.transpose() << std::endl;
			std::cout << "Pose Difference: " << (g_dRefPose - g_dVirtPose).transpose() << std::endl;
			std::cout << "Time: " << dTs << " ms." << std::endl;

            g_bLocalize = false;
        }
    }
}

// ///////////////////////////////////////////////////////////////////////////////////////
int main(
        int    argc,
        char** argv
        )
 {
    // parse arguments
    GetPot cl( argc, argv );

    // init window
    GuiWindow* pWin = new GuiWindow( 0, 0, 1024, 640, "Dense Pose Refinement" );

    // load image to be used for texture
    cv::Mat Img;

    Img = cv::imread( "antoine.jpg", 0 );

    cv::transpose( Img, Img );

    // initialize image plane
    glImgPlane.SetImage( Img.data, Img.cols, Img.rows, GL_LUMINANCE, GL_UNSIGNED_BYTE );

    Eigen::Vector6d BasePose;

    BasePose << 80, -68.1, 102.4, 0, 0, 0;
    glImgPlane.SetBaseFrame( BasePose );

    // set up image difference container
    glImgDiff.InitReset();
    glImgDiff.SetVisible();
    glImgDiff.SetPerceptable( false );


	// try mesh
    // set up mesh
    std::string sMesh = cl.follow( "CityBlock.blend", 1, "-mesh" );
	/*
    try
    {
	GLMesh glMesh;
	glMesh.Init( sMesh );
	glMesh.SetVisible();
	glMesh.SetPerceptable(true);
	glMesh.SetPose( 0, 0, 0, 180, 0, 0 );
	pWin->AddChildToRoot( &glMesh );
    }
    catch( std::exception e )
    {
        std::cerr << "Cannot load mesh. Check file exists." << std::endl;
    }
	/* */

    // register objects
    pWin->AddChildToRoot( &glImgPlane );
    pWin->AddChildToRoot( &glImgDiff );

    // prepare K matrix
    Eigen::Matrix3d dK;    // computer vision K matrix

    dK << IMG_WIDTH, 0, IMG_WIDTH / 2, 0, IMG_HEIGHT, IMG_HEIGHT / 2, 0, 0, 1;

    // initialize cameras
    RefCam.Init( &pWin->SceneGraph(), mvl::Cart2T( g_dRefPose ), dK, IMG_WIDTH, IMG_HEIGHT, eSimCamLuminance );
    VirtCam.Init( &pWin->SceneGraph(), mvl::Cart2T( g_dVirtPose ), dK, IMG_WIDTH, IMG_HEIGHT,
                  eSimCamDepth | eSimCamLuminance );

    // set up lighting
    glEnable( GL_LIGHT0 );    // activate light0
    glEnable( GL_LIGHTING );    // enable lighting

    // look at a nice place
    pWin->LookAt( -50, 40, -10, 0, 30, 0, 0, 0, -1 );

    // add our callbacks
    pWin->AddPreRenderCallback( UpdateCameraPose, NULL );
    pWin->AddPostRenderCallback( ShowCameraAndTextures, NULL );

    // launch ESM thread
    boost::thread Lclzr_Thread( Localizer );

    return(pWin->Run());
}