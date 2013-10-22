#ifndef ANDROID
#define _GLIBCXX_USE_NANOSLEEP
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

//#include <HAL/IMU/Drivers/Ninja/FtdiListener.h>

#include "Command.pb.h"


rpg::Node   Relay;

pangolin::DataLog g_PlotLogAccel;
pangolin::DataLog g_PlotLogGryo;
pangolin::DataLog g_PlotLogMag;


/*
/////////////////////////////////////////////////////////////////////////////
/// IMU callback
void IMU_Handler(pb::ImuMsg& IMUdata)
{
    if ( Relay.Write( "IMU", IMUdata ) == false ) {
        printf("NodeRelay: Error sending message.\n");
    }
}
*/


/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  pangolin::CreateWindowAndBind(__FILE__,1280,800);

  pangolin::View& imuView = pangolin::CreateDisplay().SetLayout(pangolin::LayoutEqualVertical);
  imuView.AddDisplay( pangolin::CreatePlotter("Accel", &g_PlotLogAccel) );
  imuView.AddDisplay( pangolin::CreatePlotter("Gryo", &g_PlotLogGryo) );
  imuView.AddDisplay( pangolin::CreatePlotter("Mag", &g_PlotLogMag) );

  glDisable(GL_LINE_SMOOTH);
  glDisable(GL_LIGHTING);

  GetPot clArgs(argc, argv);

//    printf("Initializing RELAY node....\n");
//    hal::IMU imu( clArgs.follow("", "-imu") );
//    imu.RegisterIMUDataCallback(IMU_Handler);


  // set up a publisher
  unsigned int nPort = clArgs.follow( 5001, "-port");
  if( Relay.Publish("IMU", nPort) == false ) {
      printf("NodeRelay: Error setting publisher.\n");
  }

//    Relay.Subscribe("CarControl", clArgs.follow("","-control" )  );

//    CommandMsg cMsg;

//    FtdiListener& Listener = FtdiListener::GetInstance();


  const double tinc = 0.01;
  double t = 0;

    for( size_t ii = 0; ; ++ii ) {

      pb::ImuMsg IMUdata;

      IMUdata.Clear();

      IMUdata.set_id(1);
      IMUdata.set_device_time( hal::Tic() );

      float st = sin(t);
      float ct = cos(t);
      float tt = tan(t);
      t += tinc;

      pb::VectorMsg* pbVec = IMUdata.mutable_accel();
      pbVec->add_data(st);
      pbVec->add_data(ct);
      pbVec->add_data(tt);

      pbVec = IMUdata.mutable_gyro();
      pbVec->add_data(ct);
      pbVec->add_data(tt);
      pbVec->add_data(st);

      pbVec = IMUdata.mutable_mag();
      pbVec->add_data(tt);
      pbVec->add_data(st);
      pbVec->add_data(ct);

      g_PlotLogAccel.Log( IMUdata.accel().data(0), IMUdata.accel().data(1), IMUdata.accel().data(2) );
      g_PlotLogGryo.Log( IMUdata.gyro().data(0), IMUdata.gyro().data(1), IMUdata.gyro().data(2) );
      g_PlotLogMag.Log( IMUdata.mag().data(0), IMUdata.mag().data(1), IMUdata.mag().data(2) );

      if ( Relay.Write( "IMU", IMUdata ) == false ) {
          printf("NodeRelay: Error sending message.\n");
      }

      /*
        cMsg.Clear();
        Relay.ReadBlocking("CarControl", cMsg);

        const int accel = int(cMsg.accel());
        const int phi = int(cMsg.phi());
        printf("Accel: %3d --- Phi: %3d\r", accel, phi );
        fflush(stdout);
        Listener.SendCommandPacket( phi, accel );
    */

      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      pangolin::FinishFrame();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }


    return 0;
}

