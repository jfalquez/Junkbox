#include <stdio.h>
#include <chrono>
#include <thread>

#include <HAL/Utils/GetPot>
#include <HAL/Utils/Node.h>
#include <HAL/IMU/IMUDevice.h>
#include <PbMsgs/Imu.pb.h>

#include <HAL/IMU/Drivers/Ninja/FtdiListener.h>

#include "Command.pb.h"


rpg::Node   Relay;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// IMU callback
void IMU_Handler(pb::ImuMsg& IMUdata)
{
    if ( Relay.Write( "IMU", IMUdata ) == false ) {
        printf("NodeRelay: Error sending message.\n");
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
    GetPot clArgs(argc, argv);

    printf("Initializing RELAY node....\n");
    hal::IMU imu( clArgs.follow("", "-imu") );
    imu.RegisterIMUDataCallback(IMU_Handler);


    // set up a publisher
    unsigned int nPort = clArgs.follow( 5001, "-port");
    if( Relay.Publish("IMU", nPort) == false ) {
        printf("NodeRelay: Error setting publisher.\n");
    }

    Relay.Subscribe("CarControl", clArgs.follow("","-control" )  );

    CommandMsg cMsg;

    FtdiListener& Listener = FtdiListener::GetInstance(); 

    for( size_t ii = 0; ; ++ii ) {

        cMsg.Clear();
        Relay.ReadBlocking("CarControl", cMsg);

        const int accel = int(cMsg.accel());
        const int phi = int(cMsg.phi());
        printf("Accel: %3d --- Phi: %3d\r", accel, phi );
        fflush(stdout);
        Listener.SendCommandPacket( phi, accel );


        //std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }


    return 0;
}

