#include <stdio.h>

#include <HAL/IMU/IMUDevice.h>
#include <PbMsgs/Imu.pb.h>

#include "Command.pb.h"
#include "FtdiDriver.h"
#include "Node.h"


FtdiDriver  g_FtdiDriver;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline double Tic()
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    return tv.tv_sec + 1e-6 * (tv.tv_usec);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// RPC call which handles setting servo positions
void ProgramControlRpc( CommandMsg& Req, CommandReply& /*Rep*/, void* /*userData*/ )
{
    printf("Received a=%.2f and p=%.2f\n",Req.accel(),Req.phi());
    fflush(stdout);
    double dAccel = std::min(500.0,std::max(0.0,Req.accel()));
    double dPhi = std::min(500.0,std::max(0.0,Req.phi()));
    g_FtdiDriver.SendCommandPacket(dPhi,dAccel);
}

rpg::Node Relay(5001);


void IMU_Handler(pb::ImuMsg& IMUdata)
{
    if ( Relay.Write( "IMU", IMUdata ) == false ) {
        printf("Error sending message.\n");
    }

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main()
{
    printf("Connecting to FTDI com port...\n");
//    g_FtdiDriver.Connect("/dev/cu.usbserial-DA009KYM");

    printf("Initializing RELAY node on port 5001....\n");
//    rpg::Node Relay(5001);
    hal::IMU imu("microstrain://");
    imu.RegisterIMUDataCallback(IMU_Handler);

    // set up a publisher
//    if( Relay.Register("ControlRpc", &ProgramControlRpc, NULL) == false ) {
//        printf("Error setting RPC callback.\n");
//    }

    // set up a publisher
    if( Relay.Publish("IMU", 5002) == false ) {
        printf("Error setting publisher.\n");
    }

    for( size_t ii = 0; ; ++ii ) {

//        SensorPacket Pkt;
//        if( g_FtdiDriver.ReadSensorPacket(Pkt) == 0 ) {
//            continue;
//        }
//        if( ii % 1000 == 0 ) {
            printf(".");
            fflush(stdout);
//        }

        sleep(1);

//        printf("AX: %6d AY: %6d AZ: %6d - GX: %6d GY: %6d GZ: %6d - MX: %6d MY: %6d MZ: %6d - ADC_LF_Y: %6d ADC_RF_Y: %6d \n",
//               Pkt.Acc_x,Pkt.Acc_y,Pkt.Acc_z,Pkt.Gyro_x,Pkt.Gyro_y,Pkt.Gyro_z,Pkt.Mag_x,Pkt.Mag_y,Pkt.Mag_z,Pkt.ADC_LF_yaw,Pkt.ADC_RF_yaw);
//        fflush(stdout);


        /*
        pb::ImuMsg pbMsg;

        pb::VectorMsg* pbVec = pbMsg.mutable_accel();
        pbVec->add_data(Pkt.Acc_x);
        pbVec->add_data(Pkt.Acc_y);
        pbVec->add_data(Pkt.Acc_z);

        pbVec = pbMsg.mutable_gyro();
        pbVec->add_data(Pkt.Gyro_x);
        pbVec->add_data(Pkt.Gyro_y);
        pbVec->add_data(Pkt.Gyro_z);

        pbVec = pbMsg.mutable_mag();
        pbVec->add_data(Pkt.Mag_x);
        pbVec->add_data(Pkt.Mag_y);
        pbVec->add_data(Pkt.Mag_z);
        */

//        Msg.set_encoderl(Pkt.Enc_LB);
//        Msg.set_encoderr(Pkt.Enc_RB);
//        Msg.set_timer(Tic());
//        if ( Relay.Write( "IMU", pbMsg ) == false ) {
//            printf("Error sending message.\n");
//        }

        sleep(1);

    }


    return 0;
}

