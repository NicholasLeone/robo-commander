#ifndef CAMERA_GIMBAL_H_
#define CAMERA_GIMBAL_H_

#include <unistd.h>		// For usleep

#include "base/peripherals.h"
#include "devices/pca9685.h"
#include "devices/tca9548a.h"
#include "sensors/bno055_i2c.h"
// #include "sensors/bno055.h"
#include "controllers/pid.h"

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
     BNO055_I2C* sensor;

     CameraGimbal();
	virtual ~CameraGimbal();

     virtual int init(COMMUNICATION_CONFIGURATION comms, int channel);
     virtual int init_sensor(COMMUNICATION_CONFIGURATION comms, TCA9548A* mux = nullptr);
     virtual int init_actuator(COMMUNICATION_CONFIGURATION comms, int channel);

     virtual void goto_neutral_state();
     virtual void updateOnce();

     void set_max_state(float value);
     void set_min_state(float value);
     void set_neutral_state(float value);
     void set_actuator_channel(int channel);

     float get_max_state();
     float get_min_state();
     float get_neutral_state();
     int get_actuator_channel();
};
#endif /** CAMERA_GIMBAL_H_ */
