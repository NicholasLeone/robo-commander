#include <iostream>
#include <unistd.h>		// For usleep
#include <pigpiod_if2.h>
#include "devices/tca9548a.h"
#include "sensors/bno055_i2c.h"

using namespace std;

int main(int argc, char *argv[]){
	float dt = 0.05;

	int pi = pigpio_start(NULL,NULL);
	cout << "[START] Multiplexer..." << endl;
	TCA9548A mux(pi, 1);
	cout << "[START] BNO-055..." << endl;
     BNO055_I2C imu1(pi, 1, 0x28, 0, &mux);
	BNO055_I2C imu2(pi, 1, 0x28, 1, &mux);

	// printf(" -------- Initializing IMU #1 ------ \r\n");
	imu1.startup();
	// printf(" -------- Initializing IMU #2 ------ \r\n");
	imu2.startup();

	printf("===========    BNO-055 Reading Euler   =================\r\n");
	float angs1[3];
	float angs2[3];
	int dummy;
	cout << "Please press [Enter] to start retrieving IMU values...";
	cin >> dummy;
	while(1){
		imu1.get_euler(&angs1[0]);
		imu2.get_euler(&angs2[0]);
		printf("IMU Angles: [#1] %.3f, %.3f, %.3f ---- [#2]: %.3f, %.3f, %.3f\r\n", angs1[0], angs1[1] , angs1[2], angs2[0], angs2[1] , angs2[2]);
		// imu.update(true);
		usleep(dt * 1000000);
	}
     return 0;
}
