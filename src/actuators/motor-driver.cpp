#include "motor-driver.h"
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>
#include <pigpiod_if2.h>
#include "pca9685/PWM-Driver-PCA9685.h"

#define MIN_PWM_US 700 //860
#define MAX_PWM_US 13000 //12600

#define PCA9685_I2C_ADD 0b10000000
#define PCA9685_I2C_BUS 1
#define NUM_PWM_CHANNELS 2

#define	INPUT			 0
#define	OUTPUT			 1
#define HIGH   1
#define LOW 0


// Directional Pins for the motors
static const int MD1_IN0 = 12;                   // RPi 3 pin header 11
static const int MD1_IN1 = 16;                   // RPi 3 pin header 12
static const int MD1_IN2 = 20;                   // RPi 3 pin header 13
static const int MD1_IN3 = 21;                   // RPi 3 pin header 15

static const int MD2_IN0 = 5;                   // RPi 3 pin header 7
static const int MD2_IN1 = 6;                   // RPi 3 pin header 16
static const int MD2_IN2 = 13;                   // RPi 3 pin header 18
static const int MD2_IN3 = 19;                   // RPi 3 pin header 22

// PWM Channel to control motor speed
static const int PCA9685_MOTOR0_CHANNEL = 0;
static const int PCA9685_MOTOR1_CHANNEL = 1;
static const int PCA9685_MOTOR2_CHANNEL = 2;
static const int PCA9685_MOTOR3_CHANNEL = 3;

static const int MAX_NUM_MOTORS = 6;
static const int MOTOR_DIRECTION_STOP = 0;
static const int MOTOR_DIRECTION_FORWARD = 1;
static const int MOTOR_DIRECTION_BACKWARD = 2;

int convertSpdRatio2Pulse(double spd_ratio);

using namespace std;

int convertSpdRatio2Pulse(double spd_ratio){
     double pulse;
     int dPulse;

     dPulse = MAX_PWM_US - MIN_PWM_US;

     // pulse = (double) (MAX_PWM_US - MIN_PWM_US) * spd_ratio + (double) MIN_PWM_US;
     pulse = (double) dPulse * spd_ratio + (double) MIN_PWM_US;
     //cout << "Converted Pulse: " << pulse << endl;
     return (int) pulse;
}

int SunfounderMotor::attachPeripheral(PERIPHERAL_PROTOCOL protocol, int channel, int id){

     if(protocol == PWM){
          this->params.pwm_channel = channel;
     }
     else if(protocol == GPIO){
          this->params.direction_gpio[id].pinNum = channel;
     }

     return 0;
}

SunfounderMotor::SunfounderMotor(int pi){
     this->pi = pi;
}

int SunfounderMotor::setMotorDirection(int direction){

     int i, temp_pin, temp_level;

     this->params.direction = direction;

     if(direction == MOTOR_DIRECTION_FORWARD){
          this->params.direction_gpio[0].pinLevel = LOW;
          this->params.direction_gpio[1].pinLevel = HIGH;
     } else if(direction == MOTOR_DIRECTION_BACKWARD) {
          this->params.direction_gpio[0].pinLevel = HIGH;
          this->params.direction_gpio[1].pinLevel = LOW;
     } else {
          this->params.direction_gpio[0].pinLevel = LOW;
          this->params.direction_gpio[1].pinLevel = LOW;
     }


     for(i = 0; i <= NUM_MOTOR_PINS-1; i++){
          temp_pin = this->params.direction_gpio[i].pinNum;
          temp_level = this->params.direction_gpio[i].pinLevel;

          gpio_write(this->pi, temp_pin, temp_level);
     }

     return 0;

}


int SunfounderMotor::setPulse(int pulse){

     int temp_channel, err;

     temp_channel = this->params.pwm_channel;
     this->params.pwm_val = pulse;

     usleep(20000);

     if((err = _outputPWM(temp_channel, pulse))){
#ifdef TEST_DEBUG
          printf("couldnt set PWM on Channel [%d]: err %d\n", temp_channel, err);
#endif
          return -5;
     }

     return 0;

}


int SunfounderMotor::setSpeed(double spd_ratio){

     int err, pulse;

     if(spd_ratio > 1.0){
	  spd_ratio = 1.0;
     } else if(spd_ratio < -1.0){
	  spd_ratio = -1.0;
     }

     // Set motor direction
     if(spd_ratio > 0){
          err = setMotorDirection(MOTOR_DIRECTION_FORWARD);
     } else if(spd_ratio < 0){
          err = setMotorDirection(MOTOR_DIRECTION_BACKWARD);
     } else {
          err = setMotorDirection(MOTOR_DIRECTION_STOP);
     }

     // Calculate the PWM Pulse value to writeOut
     pulse = convertSpdRatio2Pulse(fabs(spd_ratio));
     err = setPulse(pulse);


     return err;
}



int _initMotors(int numMotors, Motor** motors){

     int i, j, k, temp_gpio, pi, err;
     k = 0;         // Index of gpioPins[]

     int gpioPins[8] = {MD1_IN0, MD1_IN1, MD1_IN2, MD1_IN3, MD2_IN0, MD2_IN1, MD2_IN2, MD2_IN3};
     int pwmChannels[4] = {PCA9685_MOTOR0_CHANNEL, PCA9685_MOTOR1_CHANNEL, PCA9685_MOTOR2_CHANNEL, PCA9685_MOTOR3_CHANNEL};
     int i2cDevBus = PCA9685_I2C_BUS;
     int i2cDevAdd = PCA9685_I2C_ADD;

     // Initialize GPIO Pins for mCU usage
     // TODO: Use a generic scheme for initializing GPIO pins from any mCU
     pi = pigpio_start(NULL, NULL);

     // Initialize i2c comms with PWM-Driver
     if((err = _initI2c(i2cDevBus, i2cDevAdd))){
#ifdef TEST_DEBUG
          printf("couldnt initialize i2c comms: err %d\n", err);
#endif
          return -4;
     }


     for(i = 0;i<numMotors;i++){
          motors[i] = new SunfounderMotor(pi);
          // Attach Peripheral Addresses to New Motor
          motors[i]->attachPeripheral(PWM,pwmChannels[i], NULL);
          for(j = 0; j <= NUM_MOTOR_PINS - 1; j++){
               //int index = k + j;

               temp_gpio = gpioPins[k+j];
               motors[i]->attachPeripheral(GPIO, temp_gpio, j);
               //printf(" Count: %d 	Index: %d	Pin: %d \r\n", j, index, temp_gpio);
               set_mode(pi, temp_gpio, PI_OUTPUT);
          }
          // Update index of GPIO pin addresses
          k = k + NUM_MOTOR_PINS;

          // Set Initial Motor Direction
          ((SunfounderMotor*) motors[i])->setMotorDirection(MOTOR_DIRECTION_FORWARD);
          ((SunfounderMotor*) motors[i])->setSpeed(0);

     }


     return 0;
}
