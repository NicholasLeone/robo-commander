#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <istream>
#include <ostream>
#include <thread>
#include <math.h>
#include <map>

#include <pigpiod_if2.h>
#include "actuators/dc_motor.h"
#include "sensors/encoder.h"
#include "utils/utils.h"
#include "comms/udp.h"
#include "pid.h"

#define DEBUG_VERBOSE 1
#define ENCODER_A 18
#define ENCODER_B 23
#define MOTOR_DIR 20
#define MOTOR_PWM 13

using namespace std;

static int pi;
Encoder* enc;
DcMotor* motor;
UDP* rc_in;
PID* pid1;
PID_PARAMS pidM1params;

int numUpdates = 0;
int flag_start = 0;
static ofstream myFile;
static int flag_exit = 0;

static float maxSpd = 2.5;
static float curSpeed = 0;
static float curPos = 0;
static int curControl = 0;
static int maxControl = 255;
static float targetVel = -1.0;

RC_COMMAND_MSG controls;

void funDummy(int s){}

float getControl(float curVal){

	// Get current value
	float preSpd = curVal;
	float normPreSpd = preSpd / maxSpd;

	// Update Control Inputs
	float curCtrl = pid1->calculate(targetVel, normPreSpd);

	//cout << "PID CONTROL: " << curCtrl << endl;

     return curCtrl;
}

void readRC(){
     int i = 0;
     UDP* udp_line = rc_in;
     RC_COMMAND_MSG* data = &controls;

     char* dat = udp_line->read(sizeof(data)+4);

     float vel = unpackFloat(&dat[i], &i);
     float om = unpackFloat(&dat[i], &i);

     data->speed = vel;
     data->yaw = om;
	targetVel = vel;
}

void funExit(int s){

	motor->setSpeed(0);
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
     LoadInitialVariables("config.txt", variables);

     targetVel = (float) variables["target"];
     kp = (float) variables["kp"];
     ki = (float) variables["ki"];
     kd = (float) variables["kd"];
     // maxSpd = (float) variables["maxSpeed"];

	pidM1params.Kp = kp;
	pidM1params.Ki = ki;
	pidM1params.Kd = kd;

     pid1->update(pidM1params);

     //printf("VARIABLES UPDATED\r\n");
     printf("TARGET SPEED, Kp, Ki, Kd:	%.2f	%.2f	%.2f	%.2f \r\n",targetVel,kp,ki,kd);
}


int main(){

     float epsilon = 0.0001;
     float dSpd = 10;
     int i = 0;
     float speed;
     float pwm;
	int encMode = ENCODER_MODE_STEP;

	pidM1params.dt = 0.01;
	pidM1params.max = 1;
	pidM1params.min = -1;
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
          enc = new Encoder(pi, (int) ENCODER_A, (int) ENCODER_B, -1, 64, 90, 50, encMode, funDummy);
          motor = new DcMotor(pi,(int) MOTOR_PWM,(int) MOTOR_DIR);
		pid1 = new PID(pidM1params);
		rc_in = new UDP(14500,NULL);

          thread firstSpd(&enc->calc_speed,enc);
		firstSpd.detach();

          motor->setSpeed(0);

          // cout << "Pi Section" << endl;

          myFile.open("pid.csv");
          myFile << "Iteration,Target Speed,Measured Speed,PWM, Error\n";

		cout << "Waiting for User to start (Ctrl+Z)" << endl;
		while(!flag_start){
			// if(numUpdates>0){
			// 	break;
			// }
			usleep(0.1 * 1000000);
			cout << "Looping" << endl;
		}

          cout << "Beginning Loop" << endl;

          while(1){

               // cout << "Loop Section" << endl;
			// readRC();
			usleep(0.01 * 1000000);
               speed = enc->getSpeed(1);
               pwm = getControl(speed);

               // if(targetVel == 0)
               // 	motor->setSpeed(0);
               // else
          	motor->setSpeed(pwm);

			float dutycycle = pwm * maxControl;

#ifdef DEBUG_VERBOSE
               cout << "Speed, Controls, Error: " << speed/maxSpd << "		" << pwm  << "		" << pid1->_integral << endl;
               myFile << i << "," << targetVel << "," << speed/maxSpd << "," << pwm << "," << pid1->_integral << endl;
#endif

               i = i + 1;
               if(flag_exit == 1){
                    break;
               }
          }
		// myFile.close();
		// flag_exit = 1;
		//
		motor->setSpeed(0);
		printf("LOOP EXIT\r\n");
     }
     delete enc;
     delete motor;
	delete pid1;
	delete rc_in;

	pigpio_stop(pi);
     return 0;
}


/** TO COMPILE:

     g++ -w test_pid.cpp pid.cpp ../../Sensors/Encoder/encoder.cpp -o TestPID -lpigpiod_if2 -Wall -pthread
     g++ -w test_pid.cpp pid.cpp -o TestPID -lpigpiod_if2 -Wall -pthread

*/
