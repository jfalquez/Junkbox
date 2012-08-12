/*
 * File:   ImageExporter.cpp
 * Author: jmf
 *
 * Created on August 10, 2012, 2:54 PM
 */

#include "ImageExporter.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;

// //////////////////////////////////////////////////////////////////////////////
ImageExporter::ImageExporter()
{
    m_pFile = 0;
}

// //////////////////////////////////////////////////////////////////////////////
ImageExporter::~ImageExporter()
{
    if( m_pFile ) {
        m_pFile->close();
    }
}

// //////////////////////////////////////////////////////////////////////////////
void ImageExporter::Init(
        const string& sFileName
        )
{
    m_bSequential = true;
    m_sFileName   = sFileName + ".raw";
    m_pFile       = new ofstream( m_sFileName.c_str(), ios::out | ios::binary );
}

// //////////////////////////////////////////////////////////////////////////////
void ImageExporter::Init(
        const string& sFilePrefix,
        unsigned int  nImgWidth,
        unsigned int  nImgHeight
        )
{
    m_bSequential = false;
    m_nIndex      = 0;
    m_sFileName   = sFilePrefix;
    m_nImgWidth   = nImgWidth;
    m_nImgHeight  = nImgHeight;
}

// //////////////////////////////////////////////////////////////////////////////
void ImageExporter::Export(
        char*        Buff,
        unsigned int Size
        )
{
    if( m_bSequential ) {
        m_pFile->write( Buff, Size );
    } else {
        string sFileName;
        char   Index[10];

        sprintf( Index, "%05d", m_nIndex );
        m_nIndex++;

        sFileName = m_sFileName + "-" + Index + ".pgm";

        cv::Mat Img( m_nImgHeight, m_nImgWidth, CV_8UC1, Buff );
		cv::flip( Img, Img, 0 );

        cv::imwrite( sFileName, Img );
    }
}