#include <stdio.h>
#include <chrono>

#include <HAL/Utils/GetPot>
#include <HAL/IMU/IMUDevice.h>
#include <PbMsgs/Imu.pb.h>

#include "Command.pb.h"
#include "Node.h"


rpg::Node   Relay(5001);


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// RPC call which handles setting servo positions
void ProgramControlRpc( CommandMsg& Req, CommandReply& /*Rep*/, void* /*userData*/ )
{
    printf("Received a=%.2f and p=%.2f\n",Req.accel(),Req.phi());
    fflush(stdout);
    double dAccel = std::min(500.0,std::max(0.0,Req.accel()));
    double dPhi = std::min(500.0,std::max(0.0,Req.phi()));
}


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

    printf("Initializing RELAY node on port 5001....\n");
    hal::IMU imu( clArgs.follow("", "-imu") );
    imu.RegisterIMUDataCallback(IMU_Handler);

    // set up a publisher
//    if( Relay.Register("ControlRpc", &ProgramControlRpc, NULL) == false ) {
//        printf("Error setting RPC callback.\n");
//    }

    // set up a publisher
    unsigned int nPort = clArgs.follow( 5002, "-port");
    if( Relay.Publish("IMU", nPort) == false ) {
        printf("NodeRelay: Error setting publisher.\n");
    }

    for( size_t ii = 0; ; ++ii ) {

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//        sleep(1);
    }


    return 0;
}

