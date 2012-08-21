/*
 * File:   ParseArgs.h
 * Author: Juan Falquez
 *
 * Created on August 14, 2012, 11:54 AM
 */

#ifndef PARSEARGS_H
#define PARSEARGS_H

#include <RPG/Utils/GetPot>
#include <RPG/Devices/Camera/CameraDevice.h>

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
CameraDevice* ParseArgs(
        int    argc,
        char** argv
        )
{
	CameraDevice* pCam = new CameraDevice;;
    GetPot cl( argc, argv );

	// for file reader
	std::string sDeviceDriver     = cl.follow( "FileReader", 1, "-idev" );
	std::string sLeftCameraModel  = cl.follow( "lcmod.xml", 1, "-lcmod" );
	std::string sRightCameraModel = cl.follow( "rcmod.xml", 1, "-rcmod" );
	std::string sLeftFileRegex    = cl.follow( "Left.*pgm", 1, "-lfile" );
	std::string sRightFileRegex   = cl.follow( "Right.*pgm", 1, "-rfile" );
	std::string sDepthFileRegex   = cl.follow( "Depth.*pdm", 1, "-dfile" );
	std::string sSourceDir        = cl.follow( ".", 1, "-sdir"  );

	// store camera model file name
	pCam->SetProperty("CamModelFile", sSourceDir + "/" + sRightCameraModel );

	if( sDeviceDriver == "Bumblebee2" ) {
		if( sLeftCameraModel.empty() || sRightCameraModel.empty() ) {
			std::cerr << "One or more camera model files is missing!\n" << std::endl;
			std::cerr << USAGE;
			exit (0);
		}
		pCam->SetProperty("DataSourceDir", sSourceDir);
		pCam->SetProperty("CamModel-L",    sLeftCameraModel );
		pCam->SetProperty("CamModel-R",    sRightCameraModel );
	}

	if( sDeviceDriver == "FileReader" ) {
		if( sLeftCameraModel.empty() || sRightCameraModel.empty() ) {
			std::cerr << "One or more camera model files is missing!\n" << std::endl;
			std::cerr << USAGE;
			exit (0);
		}
		if( sLeftFileRegex.empty() || sRightFileRegex.empty() ) {
			std::cerr << "One or more file names is missing!\n" << std::endl;
			std::cerr << USAGE;
			exit(0);
		}
		pCam->SetProperty("DataSourceDir", sSourceDir );
		pCam->SetProperty("Channel-0",     sLeftFileRegex );
		pCam->SetProperty("Channel-1",     sRightFileRegex );
		pCam->SetProperty("Channel-2",     sDepthFileRegex );
		pCam->SetProperty("NumChannels", 3 );

	}

    // init driver
    if( !pCam->InitDriver( sDeviceDriver ) ) {
            std::cerr << "Invalid input device." << std::endl;
            exit(0);
    }
	return pCam;
}
#endif   /* PARSEARGS_H */