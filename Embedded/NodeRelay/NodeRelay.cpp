#ifndef _GLIBCXX_USE_NANOSLEEP
#define _GLIBCXX_USE_NANOSLEEP
#endif

#ifdef ANDROID
#    include <android/log.h>
#    define PrintMessage( ... ) \
          (void)__android_log_print(ANDROID_LOG_INFO,  "NodeRelay", __VA_ARGS__);
#else
#    define PrintMessage( ... ) \
          printf( __VA_ARGS__ );
#endif


#include <stdio.h>
#include <chrono>
#include <thread>

#include <pangolin/pangolin.h>

#include <HAL/Utils/GetPot>
#include <HAL/Utils/Node.h>
#include <HAL/Utils/TicToc.h>

#include <PbMsgs/Imu.pb.h>
#include <HAL/IMU/IMUDevice.h>

#include <HAL/IMU/Drivers/Ninja/FtdiListener.h>

#include "Command.pb.h"


rpg::Node   Relay;


/////////////////////////////////////////////////////////////////////////////
/// IMU callback
void IMU_Handler(pb::ImuMsg& IMUdata)
{
  if ( Relay.Write( "IMU", IMUdata ) == false ) {
    PrintMessage("NodeRelay: Error sending message.\n");
  }
}


/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  GetPot clArgs(argc, argv);

  // We need to create a window for Pangolin to handle our program entry point
  // in Android.
  pangolin::CreateWindowAndBind("NodeRelay",1280,800);
  glDisable(GL_LINE_SMOOTH);
  glDisable(GL_LIGHTING);

  ///--------------------

  PrintMessage("Setting up publisher....\n");

  // set up a publisher
  unsigned int nPort = clArgs.follow( 5001, "-port");
  if( Relay.Publish("IMU", nPort) == false ) {
      PrintMessage("NodeRelay: Error setting publisher.\n");
  }

  PrintMessage("Setting up control subscription....\n");
  Relay.Subscribe("CarControl", clArgs.follow("128.164.202.95:6001","-control" )  );

  CommandMsg cMsg;
  FtdiListener& Listener = FtdiListener::GetInstance();

  ///--------------------
#if ANDROID
  PrintMessage("Setting correct permissions....\n");
  system("su -c 'chmod 0777 /dev/ttyUSB0'");
#endif

  PrintMessage("Initializing IMU....\n");
  hal::IMU imu( clArgs.follow("ninja:///dev/ttyUSB0", "-imu") );

  PrintMessage("Registering callback....\n");
  imu.RegisterIMUDataCallback(IMU_Handler);

  ///--------------------

  for( size_t ii = 0; ; ++ii ) {

    cMsg.Clear();
    Relay.ReadBlocking("CarControl", cMsg);

    const int accel = int(cMsg.accel());
    const int phi = int(cMsg.phi());
    PrintMessage("Accel: %3d --- Phi: %3d\r", accel, phi );
    fflush(stdout);
    Listener.SendCommandPacket( phi, accel );
  }


  return 0;
}

