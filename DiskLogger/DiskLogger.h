/*
 * File:   DiskLogger.h
 * Author: guest
 *
 * Created on April 18, 2012, 4:47 PM
 */

#ifndef DISKLOGGER_H
#define DISKLOGGER_H

#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>
#include <tuple>
#include <string>
#include <condition_variable>
#include <sys/time.h>
#include <RPG/Devices/Camera/CameraDevice.h>
#include <RPG/Utils/TicToc.h>
#include <opencv2/highgui/highgui.hpp>

// //////////////////////////////////////////////////////////////////////////////
class DiskLogger
{
    public:
        DiskLogger(
                CameraDevice*      pCamera,
                const std::string& sPath,
                const std::string& sPrefixLeft,
                const std::string& sPrefixRight,
                const std::string& sSuffix,
                const bool         bConvertMono = false,
                const unsigned int nMaxBufferSize = 200
                );

        DiskLogger(
                const std::string& sCameraType,
                const std::string& sPath,
                const std::string& sPrefixLeft,
                const std::string& sPrefixRight,
                const std::string& sSuffix,
                const bool         bConvertMono = false,
                const unsigned int nMaxBufferSize = 200
                );

        ~DiskLogger();

        bool Start(
                double dDelayMS = 0
                );
        void Block();
        void Stop();

    private:
        void _DequeueAndSaveImage();
        void _GetImageAndQueue();

    private:

        // Shared values
        typedef std::tuple<rpg::ImageWrapper, std::string> ImageAndName;    // for the RGB image
        typedef std::tuple<ImageAndName, ImageAndName>     ImagePair;
        long unsigned int                                  m_nPrevTimeStampDepth;
        double                                             m_dPrevTimeStampWrite;
        double                                             m_dSumTimeWrite;
        std::deque<ImagePair>                              m_qImages;
        volatile bool                                      m_bRun;

        // Private to writer
        unsigned int m_nCount;
        unsigned int m_nSleepMS;

    private:    // Locking mechanism
        std::condition_variable m_Condition;
        std::mutex              m_Mutex, m_ConditionMutex;
        std::thread             m_ThreadSave, m_ThreadGet;

    private:
        std::string   m_sPath, m_sPrefixLeft, m_sPrefixRight, m_sSuffix, m_sCameraType;
        bool          m_bConvertMono;
        unsigned int  m_nMaxBufferSize, m_nDroppedFrames;
        CameraDevice* m_pCamera;
        double        m_dInitTime;

    public:
        static const unsigned int g_nPaddingImageNumber;
        static const unsigned int g_nPaddingCameraNumber;
};
#endif   /* DISKLOGGER_H */