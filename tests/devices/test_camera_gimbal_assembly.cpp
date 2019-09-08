#include <fstream>
#include <iostream>
#include <unistd.h>		// For usleep
#include <pigpiod_if2.h>

#include "utilities/utils.h"
#include "base/peripherals.h"
#include "devices/camera_gimbal.h"
#include "devices/tca9548a.h"

using namespace std;

CameraGimbal cg;

int numUpdates = 0;
int flag_start = 0;
static int flag_exit = 0;

static float maxVal = 180.0;
static int null_pulse = 1590;

void funDummy(int s){}
float getControl(float curVal){}

void funExit(int s){
	cg.goto_neutral_state();
	flag_exit = 1;
	usleep(1 * 1000000);
	printf("DONE\r\n");
}

void funUpdate(int s){
	flag_start = 1;
	numUpdates++;

	std::map<std::string, float> variables;
     LoadInitialVariables("/home/pi/devel/robo-commander/config/controls/camera_gimbal_pid.config", variables);

     float targetVel = (float) variables["target"];
     float kp = (float) variables["kp"];
     float ki = (float) variables["ki"];
     float kd = (float) variables["kd"];

	cg.set_p_gain(kp);			// Set P Gain
	cg.set_i_gain(ki);			// Set I Gain
	cg.set_d_gain(kd);			// Set D Gain
	cg.set_target_state(targetVel/maxVal);

     //printf("VARIABLES UPDATED\r\n");
     printf("TARGET SPEED, Kp, Ki, Kd:	%.2f	%.2f	%.2f	%.2f \r\n",targetVel,kp,ki,kd);
}


int main(){
	int i = 0;
	int bus = 1;
	int pi = pigpio_start(NULL, NULL); /* Connect to Pi. */
	if(pi < 0) return -1;

	/** Deprecated: UART Configuration for BNO-055 IMU */
	// COMMUNICATION_CONFIGURATION com;
	// com.serial.address = "/dev/serial0";
	// com.serial.baud = B115200;

	/** i2c Configuration for Gimbal Motors controlled via the PCA9685 */
	COMMUNICATION_CONFIGURATION actCommCfg;
	actCommCfg.i2c.address = 0x41;
	actCommCfg.i2c.bus = bus;
	actCommCfg.platform = pi;

	/** I2C Configuration for BNO-055 IMU */
	COMMUNICATION_CONFIGURATION imuCommCfg;
	imuCommCfg.platform = pi;
	imuCommCfg.i2c.bus = bus;
     imuCommCfg.i2c.address = 0x28;
     imuCommCfg.i2c.mux_channel = 4;

	TCA9548A mux(pi, bus);
	/** Camera Gimbal Assembly Initialization */
	// int err = cg.init(com, 3);
	int err = cg.init_actuator(actCommCfg, 3);
	err = cg.init_sensor(imuCommCfg, &mux);

	cg.gimbal->setFrequency(50);		// Set PCA9685 PWM Frequency
	cg.set_dt(0.05);				// Set PID dt
	cg.set_p_gain(10.0);			// Set P Gain
	cg.set_i_gain(3.0);				// Set I Gain
	cg.set_d_gain(0.05);			// Set D Gain
	cg.set_max_state(180.0);			// Set max angle (deg)
	cg.set_null_cmd(1590.0);			// Set command for gimbal stop
	cg.goto_neutral_state();

	/** Attach Signals */
	attach_CtrlZ(funUpdate);
	attach_CtrlC(funExit);
	cout << "Started" << endl;

	cout << "Waiting for User to start (Ctrl+Z)..." << endl;
	while(!flag_start){ usleep(0.5 * 1000000); }
	cout << "Beginning Control Loop..." << endl;

	while(1){
		cg.updateOnce();
		i = i + 1;
		if(flag_exit == 1){ break; }
	}
	cg.goto_neutral_state();
	printf("LOOP EXIT\r\n");

	return 0;
}
