#include <pangolin/pangolin.h>
#include <HAL/Utils/GetPot>
#include <HAL/IMU/IMUDevice.h>
#include <HAL/Encoder/EncoderDevice.h>

#include <HAL/Utils/Node.h>

#include "Command.pb.h"
#include "JoystickHandler.h"

pangolin::DataLog logAccel;
pangolin::DataLog logEncoder;

#define MIN_ACCEL 500
#define MAX_ACCEL 500
#define MIN_PHI 0
#define MAX_PHI 500

#define DEFAULT_ACCEL_COEF 5.65
#define DEFAULT_STEERING_COEF -650
#define DEFAULT_ACCEL_OFFSET 245.35
#define DEFAULT_STEERING_OFFSET 255

JoystickHandler theGamepad;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IMU_Handler(pb::ImuMsg& IMUdata)
{
    const pb::VectorMsg& pbAcc = IMUdata.accel();
    const float scale = 1;
    logAccel.Log( pbAcc.data(0)/scale, pbAcc.data(1)/scale, pbAcc.data(2)/scale );
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Encoder_Handler(pb::EncoderMsg& ENCdata)
{
    const float scale = 1;
    logEncoder.Log( ENCdata.data(0)/scale, ENCdata.data(1)/scale );
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
    GetPot clArgs( argc, argv );

    ///-------------------- INIT NODE
    rpg::Node Node;
    Node.Publish("CarControl", 6001);


    ///-------------------- INIT IMU
    hal::IMU theIMU( clArgs.follow("", "-imu") );
    theIMU.RegisterIMUDataCallback(IMU_Handler);


    ///-------------------- INIT Encoder
    hal::Encoder theEncoder( clArgs.follow("", "-enc") );
    theEncoder.RegisterEncoderDataCallback(Encoder_Handler);


    ///-------------------- INIT GAMEPAD
    if(theGamepad.InitializeJoystick()) {
        std::cout << "Successfully initialized gamepad." << std::endl;
    } else {
        std::cerr << "Failed to initialized gamepad." << std::endl;
    }


    ///-------------------- INIT PANGOLIN

    // Create OpenGL window in single line thanks to GLUT
    pangolin::CreateWindowAndBind("Main",640,480);
    glClearColor(0.0f,0.0f,0.0f,1.0f);

    // OpenGL 'view' of data. We might have many views of the same data.
    pangolin::Plotter thePlotter(&logAccel,0,600,-2000,15000,60,0.5);
    thePlotter.SetBounds(0.0, 1.0, 0.0, 1.0);
    pangolin::DisplayBase().AddDisplay(thePlotter);

    // Optionally add named labels
    std::vector<std::string> vLabelsAccel;
    vLabelsAccel.push_back(std::string("x"));
    vLabelsAccel.push_back(std::string("y"));
    vLabelsAccel.push_back(std::string("z"));
    logAccel.SetLabels(vLabelsAccel);

    std::vector<std::string> vLabelsEncoder;
    vLabelsEncoder.push_back(std::string("RB"));
    vLabelsEncoder.push_back(std::string("LB"));
    logEncoder.SetLabels(vLabelsEncoder);


    while(!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // update the joystick and get the accel/phi values
        theGamepad.UpdateJoystick();
//        double joystickAccel = (((double)theGamepad.GetAxisValue(1)/JOYSTICK_AXIS_MAX)*-40.0);
        double joystickAccel = (theGamepad.GetAxisValue(1) + 1 ) * (MAX_ACCEL / 2);
//        double joystickPhi = (((double)theGamepad.GetAxisValue(2)/(double)JOYSTICK_AXIS_MAX) * (MAX_SERVO_ANGLE*M_PI/180.0)*0.5);
        double joystickPhi = (-theGamepad.GetAxisValue(2) + 1) * (MAX_PHI / 2);

//        joystickAccel = joystickAccel*DEFAULT_ACCEL_COEF + DEFAULT_ACCEL_OFFSET;
//        joystickPhi = joystickPhi*DEFAULT_STEERING_COEF + DEFAULT_STEERING_OFFSET;

        printf("Accel: %3d      Phi: %3d\r",int(joystickAccel),int(joystickPhi));

        CommandMsg cMsg;
        cMsg.set_accel(joystickAccel);
        cMsg.set_phi(joystickPhi);
        Node.Write("CarControl", cMsg);

        // update pangolin GUI
        pangolin::FinishFrame();

        //sleep 0.1ms
        usleep(100);
    }
    return 0;
}

