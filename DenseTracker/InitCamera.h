/*
 * File:   InitCamera.h
 * Author: Juan Falquez
 *
 * Created on August 14, 2012, 11:54 AM
 */

#ifndef INITCAMERA_H
#define INITCAMERA_H

#include <RPG/Utils/GetPot>
#include <RPG/Devices/Camera/CameraDevice.h>

#include "Common.h"

// //////////////////////////////////////////////////////////////////////
const char USAGE[] =
"Usage:     Logger -idev <input> <options>\n"
"\n"
"where input device can be: FileReader Bumblebee2 etc\n"
"\n"
"Input Specific Options:\n"
"   FileReader:      -lfile <regular expression for left image channel>\n"
"                    -rfile <regular expression for right image channel>\n"
"                    -sf    <start frame [default 0]>\n"
"\n"
"General Options:    -lcmod <left camera model xml file>\n"
"                    -rcmod <right camera model xml file>\n"
"                    -sdir  <source directory for images and camera model files [default '.']>\n"
"\n"
"Example:\n"
"gslam  -idev FileReader  -lcmod lcmod.xml  -rcmod rcmod.xml  -lfile \"left.*pgm\"  -rfile \"right.*pgm\"\n\n";


// //////////////////////////////////////////////////////////////////////
CameraDevice* InitCamera(
        GetPot* cl
        )
{
    if( cl->search( 3, "--help", "-help", "-h" ) ) {
        std::cout << USAGE << std::endl;
        exit( 0 );
    }

    CameraDevice* pCam = new CameraDevice;

    // default parameters
    std::string sDeviceDriver     = cl->follow( "FileReader", 1, "-idev" );
    std::string sSourceDir        = cl->follow( ".", 1, "-sdir"  );
    std::string sCameraModel      = cl->follow( "cmod.xml", 1, "-cmod" );
    std::string sDepthCameraModel      = cl->follow( "cmod_d.xml", 1, "-dcmod" );
    std::string sLeftCameraModel  = cl->follow( "lcmod.xml", 1, "-lcmod" );
    std::string sRightCameraModel = cl->follow( "rcmod.xml", 1, "-rcmod" );
    std::string sLeftFileRegex    = cl->follow( "Left.*pgm", 1, "-lfile" );
    std::string sRightFileRegex   = cl->follow( "Right.*pgm", 1, "-rfile" );
    std::string sDepthFileRegex   = cl->follow( "", 1, "-dfile" );
    unsigned int nStartFrame      = cl->follow( 0, 1, "-sf"  );


    // //////////////////////////////////////////////////////////////////////
    if( sDeviceDriver == "Bumblebee2" ) {
        if( sLeftCameraModel.empty() || sRightCameraModel.empty() ) {
            std::cerr << "One or more camera model files is missing!\n" << std::endl;
            std::cerr << USAGE;
            exit (0);
        }
        pCam->SetProperty("DataSourceDir", sSourceDir);
        pCam->SetProperty("CamModel-L",    sLeftCameraModel );
        pCam->SetProperty("CamModel-R",    sRightCameraModel );
        pCam->SetProperty( "CamModFileName", sSourceDir + "/" + sRightCameraModel );
    }

    // //////////////////////////////////////////////////////////////////////
    if( sDeviceDriver == "FileReader" ) {

        pCam->SetProperty( "DataSourceDir", sSourceDir );
        pCam->SetProperty( "Channel-0",     sLeftFileRegex );
        pCam->SetProperty( "NumChannels", 2 );
        pCam->SetProperty( "StartFrame",    nStartFrame);
        pCam->SetProperty( "Loop",          true);

        // check if second image is RGB or Depth
        pCam->SetProperty( "CamModFileName", sSourceDir + "/" + sCameraModel );
        if( sDepthFileRegex.empty() ) {
            pCam->SetProperty( "Channel-1",    sRightFileRegex );
        } else {
            g_bHaveDepth = true;
            pCam->SetProperty("Channel-1",     sDepthFileRegex );
            pCam->SetProperty( "DepthCamModFileName", sSourceDir + "/" + sDepthCameraModel );
        }
        // USE THIS FOR KINECT FILES
//        g_fDepthScale = 1000.0;
    }

    // //////////////////////////////////////////////////////////////////////
    if( sDeviceDriver == "Kinect" ) {
        g_bHaveDepth = true;
        g_fDepthScale = 1000.0;
        pCam->SetProperty( "CamModFileName", sSourceDir + "/" + sCameraModel );
        bool        bGetDepth         = !cl->search( "-no-depth" );
        bool        bGetRGB           = !cl->search( "-no-rgb" );
        bool        bGetIr            = cl->search( "-with-ir" );
        bool        bAlignDepth       = cl->search( "-align-depth" );
        unsigned int    nFPS                = cl->follow( 30, 1, "-fps"  );
        std::string     sResolution         = cl->follow( "VGA", 1, "-res"  ); // follow format of XGA, SVGA, VGA, QVGA, QQVGA, etc.

        pCam->SetProperty( "GetRGB", bGetRGB );
        pCam->SetProperty( "GetDepth", bGetDepth );
        pCam->SetProperty( "GetIr", bGetIr );
        pCam->SetProperty( "AlignDepth", bAlignDepth );
        pCam->SetProperty( "FPS", nFPS );
        pCam->SetProperty( "Resolution", sResolution );
    }

    // check if -disp option was specified
    // this means that the images are disparities instead of depth maps
    g_bDisparityMaps = cl->search( "-disp" );


    // check if -aligned option was specified
    // this means that the depth and color images are aligned
    g_bAligned = cl->search( "-aligned" );


    // init driver
    std::cout << "Initializing camera driver...." << std::endl;
    if( !pCam->InitDriver( sDeviceDriver ) ) {
            std::cerr << "Invalid input device." << std::endl;
            exit(0);
    }
    std::cout << "... driver for '" << sDeviceDriver <<"' initialized successfully." << std::endl;
    return pCam;
}
#endif   /* INITCAMERA_H */
