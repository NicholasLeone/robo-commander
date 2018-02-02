#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <pigpiod_if2.h>
#include "actuators/servo.h"
#include "comms/i2c.h"

using namespace std;

#define PCA9685_I2C_ADD 0b10000000
#define PCA9685_I2C_BUS 1
#define NUM_PWM_CHANNELS 2

#define SERVO_CHANNEL_1 4
#define SERVO_CHANNEL_2 5


int Servo::attachPeripheral(PERIPHERAL_PROTOCOL protocol, int channel, int id){

     if(protocol == PWM_p){
          this->params.channel = channel;
     }
     else if(protocol == GPIO_p){}

     return 0;
}

Servo::Servo(){}

int Servo::setMaxAng(float ang){
     this->params.max_angle = ang;
     return 0;
}

int Servo::setMinAng(float ang){
     this->params.min_angle = ang;
     return 0;
}
int Servo::setMaxPulse(int pulse){
     this->params.max_pulse = pulse;
     return 0;
}
int Servo::setMinPulse(int pulse){
     this->params.min_pulse = pulse;
     return 0;
}
int Servo::setMidPulse(int pulse){
     this->params.zero_pulse = pulse;
     return 0;
}

int Servo::setPulse(int pulse){

     int temp_channel, err;

     temp_channel = this->params.channel;
     this->params.current_pulse = pulse;

     usleep(20000);

     if(err = _outputPWM(temp_channel, pulse)){
          printf("couldnt set PWM on Channel [%d]: err %d\n", temp_channel, err);
          return -5;
     }

     return 0;
}

int Servo::setAngle(float desired_angle){

     int err, pulse, my_channel;
     float my_max_angle, my_min_angle, range_ang, range_pwm, slope;
     int my_max_pwm, my_min_pwm, neutral_pwm;

     my_max_angle = this->params.max_angle;
     my_min_angle = this->params.min_angle;
     my_max_pwm = this->params.max_pulse;
     my_min_pwm = this->params.min_pulse;
     neutral_pwm = this->params.zero_pulse;

     // TODO: Fix Printf to match this case
     if(desired_angle > my_max_angle){
          printf("[INPUT-ERROR]     Required:  Input Speed Ratio <= 1.0\r\n");
          return -1;
     } else if(desired_angle < my_min_angle){
          printf("[INPUT-ERROR]     Required:  Input Speed Ratio >= -1.0\r\n");
          return -1;
     }

     range_ang = my_max_angle - my_min_angle;
     range_pwm = (float) (my_max_pwm - my_min_pwm);

     slope = range_pwm / range_ang;
     pulse = slope * desired_angle + neutral_pwm;

     // Calculate the PWM Pulse value to writeOut
     err = setPulse(pulse);

     this->params.current_angle = desired_angle;

     return 0;
}


int _initServos(int numServos, Motor** servos){

     int i, err;

     Servo* this_servo;

     int pwmChannels[2] = {SERVO_CHANNEL_1, SERVO_CHANNEL_2};
     int i2cDevBus = PCA9685_I2C_BUS;
     int i2cDevAdd = PCA9685_I2C_ADD;

     int default_mid_pulse, default_max_pulse, default_min_pulse;
     double default_max_ang, default_min_ang;

     default_max_pulse = 3500;
     default_mid_pulse = 1800;
     default_min_pulse = 900;

     default_max_ang = 90;
     default_min_ang = -90;

     // Initialize i2c comms with PWM-Driver
     if(err = _initI2c(i2cDevBus, i2cDevAdd)){
          printf("couldnt initialize i2c comms: err %d\n", err);
          return -4;
     }

     for(i = 0;i<numServos;i++){
          servos[i] = new Servo;
          this_servo = ((Servo*) servos[i]);

          // Attach Peripheral Addresses to New Servo
          servos[i]->attachPeripheral(PWM_p,pwmChannels[i], NULL);

          err = this_servo->setMaxAng(default_max_ang);
          err = this_servo->setMinAng(default_min_ang);
          err = this_servo->setMaxPulse(default_max_pulse);
          err = this_servo->setMinPulse(default_min_pulse);
          err = this_servo->setMidPulse(default_mid_pulse);
     }


     return 0;
}
