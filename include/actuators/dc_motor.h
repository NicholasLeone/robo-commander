#ifndef DC_MOTOR_H_
#define DC_MOTOR_H_

//TODO: Ensure functions (and inputs) maintain generality and modularity
//TODO: document functions w/ input params and return values

#include "base/peripherals.h"

static const int NUM_MOTOR_PINS = 2;

typedef struct MOTOR_PARAMS{
     PIN_CONFIG direction_gpio[NUM_MOTOR_PINS];
     int direction;
     int pwm_channel;
     int pwm_val;
} MOTOR_PARAMS;

class DcMotor : public Hardware{

private:
     int pi;
public:
     MOTOR_PARAMS params;

     virtual int attachPeripheral(PERIPHERAL_PROTOCOL peripheral, int channel, int id, bool verbose = false);

     DcMotor(int pi, int pwm_gpio, int dir_gpio);

     int setMotorDirection(int direction);
     int setPulse(int pulse);
     int setDuty(int duty);
     int setSpeed(float spd_ratio);
};

// int _initMotors(int numMotors, Motor** _motors);

#endif /* DC_MOTOR_H_ */
