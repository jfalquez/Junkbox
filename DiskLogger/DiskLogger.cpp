
#include "DiskLogger.h"
#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>

using namespace std;
namespace sg =SceneGraph;


extern sg::ImageView glLeftImg;
extern sg::ImageView glRightImg;
extern unsigned int g_nImgWidth;
extern unsigned int g_nImgHeight;

// //////////////////////////////////////////////////////////////////////////////
DiskLogger::DiskLogger(
        CameraDevice*      pCamera,
        const std::string& sPath,
        const std::string& sPrefixLeft,
        const std::string& sPrefixRight,
        const std::string& sSuffix,
        const bool         bConvertMono,
        const unsigned int nMaxBufferSize
        ):
    m_nPrevTimeStampDepth(
        0
        ),
    m_dPrevTimeStampWrite(
        0
        ),
    m_dSumTimeWrite(
        0
        ),
    m_bRun(
        false
        ),
    m_nCount(
        0
        ),
    m_nSleepMS(
        30
        ),
    m_sPath(
        sPath
        ),
    m_sPrefixLeft(
        sPrefixLeft
        ),
    m_sPrefixRight(
        sPrefixRight
        ),
    m_sSuffix(
        sSuffix
        ),
    m_sCameraType(
        ""
        ),
    m_bConvertMono(
        bConvertMono
        ),
    m_nMaxBufferSize(
        nMaxBufferSize
        ),
    m_nDroppedFrames(
        0
        ),
    m_dInitTime(
        Tic()
        )
{
    m_pCamera = pCamera;
}

// //////////////////////////////////////////////////////////////////////////////
DiskLogger::DiskLogger(
        const std::string& sCameraType,
        const std::string& sPath,
        const std::string& sPrefixLeft,
        const std::string& sPrefixRight,
        const std::string& sSuffix,
        const bool         bConvertMono,
        const unsigned int nMaxBufferSize
        ):
    m_nPrevTimeStampDepth(
        0
        ),
    m_dPrevTimeStampWrite(
        0
        ),
    m_dSumTimeWrite(
        0
        ),
    m_bRun(
        false
        ),
    m_nCount(
        0
        ),
    m_nSleepMS(
        30
        ),
    m_sPath(
        sPath
        ),
    m_sPrefixLeft(
        sPrefixLeft
        ),
    m_sPrefixRight(
        sPrefixRight
        ),
    m_sSuffix(
        sSuffix
        ),
    m_sCameraType(
        sCameraType
        ),
    m_bConvertMono(
        bConvertMono
        ),
    m_nMaxBufferSize(
        nMaxBufferSize
        ),
    m_nDroppedFrames(
        0
        ),
    m_pCamera(
        0
        ),
    m_dInitTime(
        Tic()
        )
{}

// //////////////////////////////////////////////////////////////////////////////
DiskLogger::~DiskLogger()
{
    Stop();
}

// //////////////////////////////////////////////////////////////////////////////
bool DiskLogger::Start(
        double dDelayMS
        )
{
    if( m_bRun ) {
        Stop();
		return false;
    }

    // only initialize camera if we have to
    if( m_pCamera == NULL ) {
		cout << "Initializing camera!" << endl;
        m_pCamera = new CameraDevice();

        if( (m_pCamera == NULL) ||!m_pCamera->InitDriver( m_sCameraType ) ) {
            return false;
        }
    }

    if( dDelayMS != 0 ) {
        usleep( dDelayMS * 1000 );
    }

    m_nDroppedFrames = 0;
    m_bRun           = true;
    m_ThreadSave     = std::thread( &DiskLogger::_DequeueAndSaveImage, this );
    m_ThreadGet      = std::thread( &DiskLogger::_GetImageAndQueue, this );
    return true;
}

// //////////////////////////////////////////////////////////////////////////////
void DiskLogger::Block()
{
    if( m_ThreadGet.joinable() ) {
        m_ThreadGet.join();
    }

    if( m_ThreadSave.joinable() ) {
        m_ThreadSave.join();
    }
}

// //////////////////////////////////////////////////////////////////////////////
void DiskLogger::Stop()
{
    m_bRun = false;

    if( m_ThreadGet.joinable() ) {
        m_ThreadGet.join();
    }

    if( m_ThreadSave.joinable() ) {
        m_ThreadSave.join();
    }
}

// //////////////////////////////////////////////////////////////////////////////
void DiskLogger::_GetImageAndQueue()
{
    std::cout << "Queue Thread started" << std::endl;

    while( m_bRun ) {
        std::stringstream ssFileNameLeft;

        ssFileNameLeft << m_sPath << "/" << m_sPrefixLeft << "_" << std::setw( g_nPaddingImageNumber )
                       << std::setfill( '0' ) << m_nCount;

        std::string       left = ssFileNameLeft.str();
        std::stringstream ssFileNameRight;

        ssFileNameRight << m_sPath << "/" << m_sPrefixRight << "_" << std::setw( g_nPaddingImageNumber )
                        << std::setfill( '0' ) << m_nCount;

        std::string right = ssFileNameLeft.str();

        if( m_qImages.size() > m_nMaxBufferSize ) {
            m_nDroppedFrames++;

            std::cout << "Dropping frame (num. dropped: " << m_nDroppedFrames << ")." << std::endl;
            {    // Lock when popping from dequeue
                std::lock_guard<std::mutex> mutex( m_Mutex );

                m_qImages.pop_back();
                m_nCount--;
            }
            continue;
        } else {
            std::vector<rpg::ImageWrapper> vImages;

            if( !m_pCamera->Capture( vImages ) ) {
                cerr << "ERROR capturing." << endl;
                continue;
            }
            ;

            if( vImages.empty() ) {
                cerr << "ERROR: empty image buffer returned from camera." << endl;
                continue;
            }

			// show left image
			glLeftImg.SetImage( vImages[0].Image.data, g_nImgWidth, g_nImgHeight, GL_RGB8, GL_RGB, GL_UNSIGNED_BYTE );

		    // show right image
			glRightImg.SetImage( vImages[1].Image.data, g_nImgWidth, g_nImgHeight, GL_INTENSITY, GL_LUMINANCE, GL_SHORT );

            m_nCount++;

            if( vImages.size() == 2 ) {
                double            dSysTime = Toc( m_dInitTime );
                long unsigned int nTimeStampRGB;

                nTimeStampRGB = m_pCamera->GetProperty<long unsigned int>( "TimestampRGB" );

                long unsigned int nTimeStampDepth;

                nTimeStampDepth = m_pCamera->GetProperty<long unsigned int>( "TimestampDepth", 0 );

                // This can be used to throw away data that has not moved forwards in time. Despite
                // the call to 'WaitAnyUpdateAll()' one of images might not have been updated (bug in driver?)

#if 0
                if( m_nPrevTimeStampDepth == nTimeStampDepth ) {
                    std::cout << "WARNING: Depth time stamp has not moved forward, dropping pair." << std::endl;

                    m_nCount--;
                    continue;
                } else {
                    m_nPrevTimeStampDepth = nTimeStampDepth;
                }
#endif

                ssFileNameLeft << m_sSuffix;
                ssFileNameRight << m_sSuffix;

                rpg::ImageWrapper im1;

                im1.Image = vImages[0].Image.clone();

                // cout << "Time: " << nTimeStampRGB << endl;
                im1.Map.SetProperty( "CameraTime", nTimeStampRGB );
                im1.Map.SetProperty( "SystemTime", dSysTime );

                rpg::ImageWrapper im2;

                im2.Image = vImages[1].Image.clone();

                im2.Map.SetProperty( "CameraTime", nTimeStampDepth );
                im2.Map.SetProperty( "SystemTime", dSysTime );
                {
                    std::lock_guard<std::mutex> mutex( m_Mutex );

                    m_qImages.push_back( ImagePair( ImageAndName( im1, ssFileNameLeft.str() ),
                                                    ImageAndName( im2, ssFileNameRight.str() ) ) );
                }

                // do not sleep as our call actually blocks
                // usleep(m_nSleepMS * 1000);
            } else {
                m_nCount--;

                cerr << "ERROR: expecting 2 images: RGB and depth, got: " << vImages.size() << " images." << endl;
            }

            m_Condition.notify_one();

#if 0
            if( m_nCount > 200 ) {    // FOR DEBUGGING
                m_bRun = false;
            }
#endif

        }
    }

    m_Condition.notify_one();
	cout << "Enqueing thread died!!!!!" << endl;

}

// //////////////////////////////////////////////////////////////////////////////
void DiskLogger::_DequeueAndSaveImage()
{
    std::cout << "Dequeue Thread started" << std::endl;

    m_dPrevTimeStampWrite = Tic();

    std::unique_lock<std::mutex> cond( m_ConditionMutex );

    while( m_bRun ) {
        if( m_qImages.empty() ) {
//            std::cout << "Empty" << std::endl;
            // Wait for signal
            m_Condition.wait( cond );
        }

//        std::cout << "Not empty" << std::endl;
        while( !m_qImages.empty() ) {
            ImagePair ImagePair;

            // Lock when popping from dequeue
            {
                std::lock_guard<std::mutex> mutex( m_Mutex );

                ImagePair = m_qImages.front();

                m_qImages.pop_front();
            }

            std::cout << m_qImages.size() << " " << std::get<1>( std::get<0>( ImagePair ) ) << " "
                      << std::get<1>( std::get<1>( ImagePair ) ) << std::endl;

            std::string sImageNameRGB          = std::get<1>( std::get<0>( ImagePair ) );
            std::string sImageNameDepth        = std::get<1>( std::get<1>( ImagePair ) );
            std::string sExtraImageInfoNameRGB = sImageNameRGB;

            sExtraImageInfoNameRGB.erase( sExtraImageInfoNameRGB.rfind( '.' ) );

            sExtraImageInfoNameRGB += ".txt";

            std::string sExtraImageInfoNameDepth = sImageNameDepth;

            sExtraImageInfoNameDepth.erase( sExtraImageInfoNameDepth.rfind( '.' ) );

            sExtraImageInfoNameDepth += ".txt";
            m_dSumTimeWrite          += Toc( m_dPrevTimeStampWrite );

            cout << "FPS: " << 1 / Toc( m_dPrevTimeStampWrite ) << endl;

            m_dPrevTimeStampWrite = Tic();

            cout << "FPS: " << double(m_nCount) / m_dSumTimeWrite << endl;

            // Save

#if 0
            cv::imwrite( sImageNameRGB.c_str(), std::get<0>( std::get<0>( ImagePair ) ).Image );
            cv::imwrite( sImageNameDepth.c_str(), std::get<0>( std::get<1>( ImagePair ) ).Image );
#else
            std::get<0>( std::get<0>( ImagePair ) ).write( sImageNameRGB );
            std::get<0>( std::get<1>( ImagePair ) ).write( sImageNameDepth );
#endif

#if 0
            cv::imwrite( sImageNameRGB.c_str(), std::get<0>( std::get<0>( ImagePair ) ).Image );
            cv::imwrite( sImageNameDepth.c_str(), std::get<0>( std::get<1>( ImagePair ) ).Image );

            // #else
            IplImage mIm = std::get<0>( std::get<0>( ImagePair ) ).Image;

            std::fstream f( sImageNameRGB.c_str(), ios::out | ios::binary );

            // f.write( mIm.imageData, mIm.widthStep * mIm.height * 3 );
            f.write( mIm.imageData, mIm.width * mIm.height * 3 );

            // cout << mIm.widthStep << endl;
            // cout << mIm.width << endl;
            // cout << mIm.height << endl;
            f.close();
#endif

        }
    }
	cout << "Dequeing thread died!!!!!" << endl;
}