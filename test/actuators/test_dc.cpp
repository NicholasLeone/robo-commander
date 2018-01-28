#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <pigpiod_if2.h>
#include "dc_motor.h"

using namespace std;

float linSpeed = 0;
float angSpeed = 0;
float ang1 = 0;
float ang2 = 0;

int main(int argc, char *argv[]){
     int err;

     int pi = pigpio_start(NULL,NULL);
     DcMotor FR(pi,12,16);
     FR.setSpeed(1.0);
     sleep(1);
     FR.setSpeed(-1.0);
     sleep(1);
     FR.setSpeed(0);

     return 0;
}


/** TO COMPILE:
     g++ test_motors.cpp motor-driver.cpp -o TestMotor -lpigpiod_if2 -Wall -pthread
     g++ test_motors.cpp motor-driver.cpp ./pca9685/PWM-Driver-PCA9685.cpp ./pca9685/pwm-pca9685-user.c -o TestRC -lpigpiod_if2 -Wall -pthread -std=c++11
*/
