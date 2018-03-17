#ifndef SWANSONV2_H_
#define SWANSONV2_H_

#include "comms/udp.h"
#include "profiles/dual_roboclaw.h"

class SwansonV2 {

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

     DualClaw* claws;

     // FUNCTIONS
	SwansonV2(int pi);
     ~SwansonV2();

     void readRC();
     void drive(float v, float w);
     void updateSensors();
};


#endif // SWANSONV2_H_
