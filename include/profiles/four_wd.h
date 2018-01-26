#ifndef FOURWD_H_
#define FOURWD_H_

#include <stdarg.h>
#include <string>
#include "../../include/base/params.h"
#include "../../Actuators/DC_Motor/dc_motor.h"
#include "../../Sensors/Encoder/encoder.h"
#include "../../Communications/UDP/udp.h"
#include "../../Actuators/RoboClaw/roboclaw.h"
#include "../PID/pid.h"

/** Mr. Swanson Config */

/* Front Right Motor:
*         Direction Pin - 26
*         Speed Pin     - 19
*         Encoder A     - 27
*         Encoder B     - 22
*/
/* Rear Right Motor:
*         Direction Pin - 13
*         Speed Pin     - 6
*         Encoder A     - 4
*         Encoder B     - 17
*/
/* Front Left Motor:
*         Direction Pin - 21
*         Speed Pin     - 20
*         Encoder A     - 24
*         Encoder B     - 25
*/
/* Rear Left Motor:
*         Direction Pin - 16
*         Speed Pin     - 12
*         Encoder A     - 18
*         Encoder B     - 23
*/
/** END MR. SWANSON CONFIG */


class FourWD {

private:

     int _pi;
     int _port;
     int _mode;
     int ser_handle;

     float max_speed;
     float centerToWheelRadius;

     float target_speed_left;
     float target_speed_right;

     float measured_speed_front_left;
     float measured_speed_front_right;
     float measured_speed_rear_left;
     float measured_speed_rear_right;

     int measured_pos_front_left;
     int measured_pos_front_right;
     int measured_pos_rear_left;
     int measured_pos_rear_right;

     void _driveLeft(float speed);
     void _driveRight(float speed);

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

     RoboClaw* leftclaw;
     RoboClaw* rightclaw;

     PID* pidLeft;
     PID* pidRight;

     PID_PARAMS pidParamsLeft;
     PID_PARAMS pidParamsRight;

     // FUNCTIONS
	FourWD(int pi);
     ~FourWD();

     void readRC();
     void drive(float v, float w);
     float calculateControlSpeed(PID* _pid, float target, float measurement);
     void updateSensors();
};



#endif // FOURWD_H_
