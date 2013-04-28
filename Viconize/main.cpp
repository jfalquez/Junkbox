#include <iostream>
#include <iomanip>

#include <Eigen/Dense>
#include <opencv.hpp>

#include <RPG/Utils/GetPot>
#include <RPG/Utils/InitCam.h>
#include <RPG/Devices/Camera/CameraDevice.h>

#include <Mvlpp/Mvl.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    std::cout << "Running... " << std::endl;

    GetPot clArgs(argc, argv);

    CameraDevice Cam;

    rpg::InitCam( Cam, clArgs );

    const unsigned int nStartFrame = Cam.GetProperty( "StartFrame", 0 ) + 1;

    // container for images
    std::vector< rpg::ImageWrapper > vImages;


    std::string sSrcDir = Cam.GetProperty( "DataSourceDir", "." );

    std::string sViconFile = clArgs.follow( "vicon.txt", "-vicon" );
    std::string sViconFullFile = sSrcDir + "/" + sViconFile;

    std::ifstream ViconFile;
    ViconFile.open( sViconFullFile.c_str() );
    if( ViconFile.is_open() == false ) {
        std::cerr << "error: Could not open Vicon file!" << std::endl;
        exit(1);
    }


    double                      CurViconTime;
    Eigen::Vector6d             CurViconPose;
    double                      PrevViconTime = 0;
    Eigen::Vector6d             PrevViconPose;


    std::string sValue;

    getline( ViconFile, sValue, ',' );
    CurViconTime = atof( sValue.c_str() );
    getline( ViconFile, sValue, ',' ); // local vicon time
    getline( ViconFile, sValue, ',' );
    CurViconPose(0) = atof( sValue.c_str() );
    getline( ViconFile, sValue, ',' );
    CurViconPose(1) = atof( sValue.c_str() );
    getline( ViconFile, sValue, ',' );
    CurViconPose(2) = atof( sValue.c_str() );
    getline( ViconFile, sValue, ',' );
    CurViconPose(3) = atof( sValue.c_str() );
    getline( ViconFile, sValue, ',' );
    CurViconPose(4) = atof( sValue.c_str() );
    getline( ViconFile, sValue );
    CurViconPose(5) = atof( sValue.c_str() );


    for( unsigned int nFrame = nStartFrame; Cam.Capture(vImages); nFrame++ ) {
        double ImageTime = vImages[0].Map.GetProperty( "SystemTime", 0.0 );
        if( ImageTime == 0 ) {
            std::cerr << "error: No time found in image property map! Aborting." << std::endl;
            exit(1);
        }

        while( CurViconTime < ImageTime ) {

            PrevViconTime = CurViconTime;
            PrevViconPose = CurViconPose;

            getline( ViconFile, sValue, ',' );
            CurViconTime = atof( sValue.c_str() );
            getline( ViconFile, sValue, ',' ); // local vicon time
            getline( ViconFile, sValue, ',' );
            CurViconPose(0) = atof( sValue.c_str() );
            getline( ViconFile, sValue, ',' );
            CurViconPose(1) = atof( sValue.c_str() );
            getline( ViconFile, sValue, ',' );
            CurViconPose(2) = atof( sValue.c_str() );
            getline( ViconFile, sValue, ',' );
            CurViconPose(3) = atof( sValue.c_str() );
            getline( ViconFile, sValue, ',' );
            CurViconPose(4) = atof( sValue.c_str() );
            getline( ViconFile, sValue );
            CurViconPose(5) = atof( sValue.c_str() );

            if( ViconFile.eof() ) {
                std::cerr << "warning: Ran out of Vicon entries! Exit." << std::endl;
                ViconFile.close();
                exit(1);
            }
        }

        if( PrevViconTime == 0 ) {
            std::cerr << "error: The time of first image has no Vicon entries! Aborting." << std::endl;
            exit(1);
        }


        // interpolate vicon poses
        Eigen::Matrix3d Roti = mvl::Cart2R( PrevViconPose.tail(3) );
        Eigen::Matrix3d Rotf = mvl::Cart2R( CurViconPose.tail(3) );
        Eigen::Quaterniond Qi(Roti);
        Eigen::Quaterniond Qf(Rotf);

        // find scaleing value
        double deltaViconTime = CurViconTime - PrevViconTime;
        double deltaImageTime = ImageTime - PrevViconTime;
        double Scale = deltaImageTime / deltaViconTime;

        Eigen::Quaterniond iQ = Qf.slerp( Scale, Qi );
        Eigen::Matrix3d iRot = iQ.matrix();

        Eigen::Vector6d deltaViconPose = CurViconPose - PrevViconPose;
        Eigen::Vector6d iViconPose = PrevViconPose + (deltaViconPose * Scale);
        iViconPose.tail(3) = mvl::R2Cart( iRot );


        // put pose in property map
        vImages[0].Map.SetProperty("ViconPoseX", iViconPose(0) );
        vImages[0].Map.SetProperty("ViconPoseY", iViconPose(1) );
        vImages[0].Map.SetProperty("ViconPoseZ", iViconPose(2) );
        vImages[0].Map.SetProperty("ViconPoseP", iViconPose(3) );
        vImages[0].Map.SetProperty("ViconPoseQ", iViconPose(4) );
        vImages[0].Map.SetProperty("ViconPoseR", iViconPose(5) );

        // generate meta data file name
        char            Index[10];
        sprintf( Index, "%05d", nFrame );
        std::string FilePrefix = "Left-";
        std::string FileName;
        FileName = FilePrefix + Index + ".txt";


        // export property map
        cv::FileStorage oFile( FileName.c_str(), cv::FileStorage::WRITE );
        if( !oFile.isOpened() ) {
            std::cerr << "error: Could not open export file!" << std::endl;
            exit(1);
        }

        std::map<std::string,std::string>& ppMap = vImages[0].Map.GetPropertyMap();
        for( std::map<std::string,std::string>::iterator it = ppMap.begin(); it != ppMap.end(); it++ ) {
            oFile << it->first << it->second;
        }

        oFile.release();
    }

    // close file
    ViconFile.close();

    std::cout << "Done!" << std::endl;

    return 0;
}
