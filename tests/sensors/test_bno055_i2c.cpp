#include <iostream>
#include <unistd.h>		// For usleep
#include <pigpiod_if2.h>
#include "sensors/bno055_i2c.h"
using namespace std;

int main(int argc, char *argv[]){
	float dt = 0.05;

	int pi = pigpio_start(NULL,NULL);
	cout << "[START] BNO-055..." << endl;
     BNO055_I2C imu(pi, 1, 0x28);
	imu.startup(true);

	printf("===========    BNO-055 Reading Euler   =================\r\n");
	float angs[3];
	int dummy;
	cout << "Please press [Enter] to start retrieving IMU values...";
	cin >> dummy;
	while(1){
		imu.get_euler(&angs[0]);
		printf("Euler Angles: %f, %f, %f\r\n", angs[0], angs[1] , angs[2]);
		// imu.update(true);
		usleep(dt * 1000000);
	}
     return 0;
}
