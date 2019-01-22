#ifndef CAMERA_GIMBAL_H_
#define CAMERA_GIMBAL_H_

#include "base/peripherals.h"
#include "devices/pca9685.h"
#include "sensors/bno055.h"
#include "controls/pid.h"

class CameraGimbal : public PID{
private:
     COMMUNICATION_CONFIGURATION _comms;
     int actuator_channel;
     int loopCount = 0;
     float max_state;
     float min_state;
     float neutral_state;
public:
     PCA9685* gimbal;
     BNO055 sensor;

     CameraGimbal();
	virtual ~CameraGimbal();

     virtual int init(COMMUNICATION_CONFIGURATION comms, int channel);
     virtual int _init_sensor(COMMUNICATION_CONFIGURATION comms);
     virtual int _init_actuator(COMMUNICATION_CONFIGURATION comms, int channel);

     virtual void goto_neutral_state();
     virtual void updateOnce();

     void set_max_state(float value);
     void set_min_state(float value);
     void set_neutral_state(float value);
};
#endif /** CAMERA_GIMBAL_H_ */
