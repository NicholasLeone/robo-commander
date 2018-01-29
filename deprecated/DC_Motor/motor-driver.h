#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

//TODO: Ensure functions (and inputs) maintain generality and modularity
//TODO: document functions w/ input params and return values

#include <stdint.h>
#include "motors.h"

static const int NUM_MOTOR_PINS = 2;

class SunfounderMotor : public Motor {

private:
     int pi;
public:
     MOTOR_PARAMS params;

     virtual int attachPeripheral(PERIPHERAL_PROTOCOL peripheral, int channel, int id);

     SunfounderMotor(int pi);

     int setMotorDirection(int direction);
     int setPulse(int pulse);
     int setSpeed(double spd_ratio);
};


int _initMotors(int numMotors, Motor** _motors);



#endif /* MOTOR_DRIVER_H_ */
