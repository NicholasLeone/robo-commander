#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>

#include <pigpiod_if2.h>
#include "dc_motor.h"
#include "utils/utils.h"

using namespace std;

#define	INPUT       0
#define	OUTPUT      1
#define   HIGH        1
#define   LOW         0

static const int MAX_NUM_MOTORS = 6;
static const int MOTOR_DIRECTION_STOP = 0;
static const int MOTOR_DIRECTION_FORWARD = 1;
static const int MOTOR_DIRECTION_BACKWARD = 2;

#define MAX_PWM_US 2000
#define MIN_PWM_US 1000
#define MID_PWM_US 1500

int DcMotor::attachPeripheral(PERIPHERAL_PROTOCOL protocol, int channel, int id){

     if(protocol == PWM_p){
          this->params.pwm_channel = channel;
     }
     else if(protocol == GPIO_p){
          this->params.direction_gpio[id].pinNum = channel;
     }

     return 0;
}

DcMotor::DcMotor(int pi, int pwm_gpio, int dir_gpio){
     this->pi = pi;
     this->attachPeripheral(GPIO_p, dir_gpio,0);
     this->attachPeripheral(PWM_p,pwm_gpio,NULL);
}

int DcMotor::setMotorDirection(int direction){

     int i, temp_pin, temp_level;

     this->params.direction = direction;

     if(direction == MOTOR_DIRECTION_FORWARD){
          this->params.direction_gpio[0].pinLevel = LOW;
     } else if(direction == MOTOR_DIRECTION_BACKWARD) {
          this->params.direction_gpio[0].pinLevel = HIGH;
     } else {
          this->params.pwm_val = 0;
          this->setPulse(0);
     }

     temp_pin = this->params.direction_gpio[0].pinNum;
     temp_level = this->params.direction_gpio[0].pinLevel;
     gpio_write(this->pi, temp_pin, temp_level);

     return 0;
}


int DcMotor::setPulse(int pulse){

     int temp_pwm = this->params.pwm_channel;
     this->params.pwm_val = pulse;
     int err = set_servo_pulsewidth(this->pi,temp_pwm, pulse);

     if(err < 0){
          printf("couldnt set PWM on Channel [%d]: err %d\n", temp_pwm, err);
          return err;
     }

     return 0;
}

int DcMotor::setDuty(int duty){

     int temp_pwm = this->params.pwm_channel;
     this->params.pwm_val = duty;
     int err = set_PWM_dutycycle(this->pi,temp_pwm, duty);

     if(err < 0){
          printf("couldnt set PWM on Channel [%d]: err %d\n", temp_pwm, err);
          return err;
     }

     return 0;
}

int DcMotor::setSpeed(float spd_ratio){

     int err;

     if(spd_ratio > 1.0)
	  spd_ratio = 1.0;
     else if(spd_ratio < -1.0)
	  spd_ratio = -1.0;

     // Set motor direction
     if(spd_ratio > 0)
          err = setMotorDirection(MOTOR_DIRECTION_FORWARD);
     else if(spd_ratio < 0)
          err = setMotorDirection(MOTOR_DIRECTION_BACKWARD);
     else
          err = setMotorDirection(MOTOR_DIRECTION_STOP);

     // Calculate the PWM Duty value to writeOut
     int duty = fabs(spd_ratio) * 255;
     err = setDuty(duty);

     // Calculate the PWM pulsewidth value to writeOut
     // int pulse = convertSpdRatio2Pulse(spd_ratio,MAX_PWM_US, MIN_PWM_US, MID_PWM_US);
     // err = setPulse(pulse);
     return err;
}



// int _initMotors(int numMotors, Motor** motors){
//
//      int i, j, k, temp_gpio, pi, err;
//      k = 0;         // Index of gpioPins[]
//
//      int gpioPins[8] = {MD1_IN0, MD1_IN1, MD1_IN2, MD1_IN3, MD2_IN0, MD2_IN1, MD2_IN2, MD2_IN3};
//      int pwmChannels[4] = {PCA9685_MOTOR0_CHANNEL, PCA9685_MOTOR1_CHANNEL, PCA9685_MOTOR2_CHANNEL, PCA9685_MOTOR3_CHANNEL};
//      int i2cDevBus = PCA9685_I2C_BUS;
//      int i2cDevAdd = PCA9685_I2C_ADD;
//
//      // Initialize GPIO Pins for mCU usage
//      // TODO: Use a generic scheme for initializing GPIO pins from any mCU
//      pi = pigpio_start(NULL, NULL);
//
//      // Initialize i2c comms with PWM-Driver
//      if((err = _initI2c(i2cDevBus, i2cDevAdd))){
// #ifdef TEST_DEBUG
//           printf("couldnt initialize i2c comms: err %d\n", err);
// #endif
//           return -4;
//      }
//
//
//      for(i = 0;i<numMotors;i++){
//           motors[i] = new SunfounderMotor(pi);
//           // Attach Peripheral Addresses to New Motor
//           motors[i]->attachPeripheral(PWM,pwmChannels[i], NULL);
//           for(j = 0; j <= NUM_MOTOR_PINS - 1; j++){
//                //int index = k + j;
//
//                temp_gpio = gpioPins[k+j];
//                motors[i]->attachPeripheral(GPIO, temp_gpio, j);
//                //printf(" Count: %d 	Index: %d	Pin: %d \r\n", j, index, temp_gpio);
//                set_mode(pi, temp_gpio, PI_OUTPUT);
//           }
//           // Update index of GPIO pin addresses
//           k = k + NUM_MOTOR_PINS;
//
//           // Set Initial Motor Direction
//           ((SunfounderMotor*) motors[i])->setMotorDirection(MOTOR_DIRECTION_FORWARD);
//           ((SunfounderMotor*) motors[i])->setSpeed(0);
//
//      }
//
//
//      return 0;
// }
