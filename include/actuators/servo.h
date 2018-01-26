#ifndef SERVO_H_
#define SERVO_H_

#include <stdint.h>
#include "../../include/params.h"
#include "../Motors/motors.h"
#include "../Motors/pca9685/PWM-Driver-PCA9685.h"



class Servo : public Motor{

public:

     virtual int attachPeripheral(PERIPHERAL_PROTOCOL peripheral, int channel, int id);
     Servo();
     ~Servo();

     SERVO_PARAMS params;

     int setMaxAng(float ang);
     int setMinAng(float ang);
     int setMaxPulse(int pulse);
     int setMinPulse(int pulse);
     int setMidPulse(int pulse);

     int setPulse(int pulse);
     int setAngle(float desired_angle);

};

int _initServos(int numServos, Motor** _servos);

#endif
