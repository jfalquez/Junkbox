/*
 * File:   ImageExporter.h
 * Author: jmf
 *
 * Created on August 10, 2012, 2:54 PM
 */

#ifndef IMAGEEXPORTER_H
#define IMAGEEXPORTER_H

#include <string>
#include <fstream>

class ImageExporter
{
    public:
        ImageExporter();

        ~ImageExporter();

		// this method is used for sequential writing
        void Init(
                const std::string& sFileName
                );

		// this method is used for per-image writing
        void Init(
                const std::string& sFilePrefix,
                unsigned int       nImgWidth,
                unsigned int       nImgHeight
                );

		// export image
        void Export(
                char*        Buff,
                unsigned int Size = 0
                );

    private:
        bool           m_bSequential;
        std::string    m_sFileName;
        std::ofstream* m_pFile;			// used for sequential writing
        unsigned int   m_nIndex;		// used for per-image writing
        unsigned int   m_nImgWidth;
        unsigned int   m_nImgHeight;
};
#endif   /* IMAGEEXPORTER_H */