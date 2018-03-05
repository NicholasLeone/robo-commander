#ifndef SWANSON_H_
#define SWANSON_H_


#include <stdarg.h>
#include <string>
#include "base/params.h"
#include "actuators/dc_motor.h"
#include "sensors/encoder.h"
#include "comms/udp.h"
#include "devices/roboclaw.h"
#include "controls/pid.h"

class Swanson {

private:

     int _pi;
     int _port;
     int _mode;
     int ser_handle;

     float max_speed;
     float centerToWheelRadius;

public:

     UDP* rc_in;
     RC_COMMAND_MSG controls;

     DcMotor* FR;
     DcMotor* FL;
     DcMotor* RR;
     DcMotor* RL;

     Encoder* FR_enc;
     Encoder* FL_enc;
     Encoder* RR_enc;
     Encoder* RL_enc;

     PID* pidLeft;
     PID* pidRight;

     PID_PARAMS pidParamsLeft;
     PID_PARAMS pidParamsRight;

     RoboClaw* leftclaw;
     RoboClaw* rightclaw;

     // FUNCTIONS
	Swanson(int pi);
     ~Swanson();

     void readRC();
     void drive(float v, float w);
     void updateSensors();
};


#endif // SWANSON_H_
