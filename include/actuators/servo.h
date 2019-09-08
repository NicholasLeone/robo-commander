#ifndef SERVO_H_
#define SERVO_H_

#include "base/peripherals.h"

typedef struct SERVO_PARAMS{
     int channel;
     float max_angle;
     float min_angle;
     float current_angle;
     int max_pulse;
     int min_pulse;
     int zero_pulse;
     int current_pulse;
} SERVO_PARAMS;

class Servo : public Motor{

public:

     virtual int attachPeripheral(PERIPHERAL_PROTOCOL peripheral, int channel, int id, bool verbose = false);
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

/* Placeholder functions just to allow deprecated code to build */
int _initI2c(int i2cBus, int address);
int _outputPWM(int channel, int val);

#endif
