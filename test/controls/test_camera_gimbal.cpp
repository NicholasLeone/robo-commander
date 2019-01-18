#include <fstream>
#include <iostream>

#include "devices/pca9685.h"
#include "sensors/bno055.h"
#include "utils/utils.h"
#include "pid.h"
#include <pigpiod_if2.h>

#define R2D(angleRadians) ((angleRadians) * 180.0 / M_PI)

#define DEBUG_VERBOSE 0

using namespace std;

static int pi;
PCA9685* gimbal;
PID* pid1;
PID_PARAMS pidM1params;

int numUpdates = 0;
int flag_start = 0;
static ofstream myFile;
static int flag_exit = 0;

static float maxVal = 180.0;
static int maxControl = 2000;
static float targetVel = -1.0;
static int null_pulse = 1590;
static int motor_channel = 3;

void funDummy(int s){}

float getControl(float curVal){

	// Get current value
	// float preSpd = curVal;
	float normVal = curVal / maxVal;

	// Update Control Inputs
	float curCtrl = pid1->calculate(normVal);

	//cout << "PID CONTROL: " << curCtrl << endl;

     return curCtrl;
}

void funExit(int s){

	gimbal->setPulsewidth(motor_channel,null_pulse);
	myFile.close();
	flag_exit = 1;
	usleep(1 * 1000000);
	printf("DONE\r\n");
}

void funUpdate(int s){

     float kp,ki,kd;
	flag_start = 1;
	numUpdates++;

	std::map<std::string, float> variables;
     LoadInitialVariables("/home/hunter/devel/robo-dev/config/controls/camera_gimbal_pid.config", variables);

     targetVel = (float) variables["target"];
     kp = (float) variables["kp"];
     ki = (float) variables["ki"];
     kd = (float) variables["kd"];

	pidM1params.Kp = kp;
	pidM1params.Ki = ki;
	pidM1params.Kd = kd;

     pid1->set_params(pidM1params);
	pid1->set_target(targetVel/maxVal);

     //printf("VARIABLES UPDATED\r\n");
     printf("TARGET SPEED, Kp, Ki, Kd:	%.2f	%.2f	%.2f	%.2f \r\n",targetVel,kp,ki,kd);
}


int main(){

     int i = 0;
     float angle, pwm;
	float dt = 0.05;
	float angles[3];
	/** UART Configuration for BNO-055 IMU */
	string device = "/dev/serial0";
	int baud = B115200;


	/** i2c Configuration for Gimbal Motors controlled via the PCA9685 */
	int bus = 1;
     int add = 0x70;
	int freq = 50;

	/** PID Initial Configuration */
	pidM1params.dt = 0.01;
	pidM1params.max = 0.3;
	pidM1params.min = -0.3;
	pidM1params.pre_error = 0;
	pidM1params.integral = 0;
	pidM1params.Kp = 1.0;
	pidM1params.Ki = 1.0;
	pidM1params.Kd = 1.0;

     attach_CtrlZ(funUpdate);
     attach_CtrlC(funExit);

     pi = pigpio_start(NULL, NULL); /* Connect to Pi. */
     cout << "Started" << endl;

     if(pi >= 0){
                /** BNO-055 IMU Start-Up */
		BNO055 imu(device, baud);
		int err = imu.begin();
		if(err < 0)
			printf("[ERROR] BNO055::begin] ---- %d.\r\n", err);
		else
			printf("[SUCCESS] BNO-055 Initialized \r\n\r\n");
		pid1 = new PID(pidM1params);
		pid1->set_dt(dt);

		gimbal = new PCA9685(pi, bus, add);
          gimbal->setFrequency(freq);
		gimbal->setPulsewidth(motor_channel,null_pulse);
          // myFile.open("pid.csv");
          // myFile << "Iteration,Target Speed,Measured Speed,PWM, Error\n";

		cout << "Waiting for User to start (Ctrl+Z)..." << endl;
		while(!flag_start){
			usleep(0.5 * 1000000);
			// cout << "Looping" << endl;
		}

          cout << "Beginning Control Loop..." << endl;

          while(1){
			imu.get_euler(&angles[0]);
			angle = angles[1];
			pwm = getControl(angle);
			usleep(dt);
			pwm = pwm * maxControl + null_pulse;
          		gimbal->setPulsewidth(motor_channel,(int)pwm);



			#ifdef DEBUG_VERBOSE
               cout << "Angle, Controls, Error: " << angle << "		" << pwm  << "		" << pid1->_integral << endl;
               // myFile << i << "," << targetVel << "," << speed/maxSpd << "," << pwm << "," << pid1->_integral << endl;
			#endif

               i = i + 1;
               if(flag_exit == 1){
                    break;
               }
          }
		// myFile.close();
		// flag_exit = 1;
		//
		gimbal->setPulsewidth(motor_channel,null_pulse);
		printf("LOOP EXIT\r\n");
     }
     delete gimbal;
	delete pid1;

	pigpio_stop(pi);
     return 0;
}


/** TO COMPILE:

     g++ -w test_pid.cpp pid.cpp ../../Sensors/Encoder/encoder.cpp -o TestPID -lpigpiod_if2 -Wall -pthread
     g++ -w test_pid.cpp pid.cpp -o TestPID -lpigpiod_if2 -Wall -pthread

*/
