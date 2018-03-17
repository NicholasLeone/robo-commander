#include <stdio.h>
#include <string.h>
#include <iostream>
#include <math.h>

#include "imu.h"

#define R2D(angleRadians) ((angleRadians) * 180.0 / M_PI)

using namespace std;

int main(int argc, char *argv[]){

	float angle;
	int dt;

	string path = "/home/hunter/devel/robo-dev/config/sensors";
	string file = "mpu9250";

     IMU imu(path, file);

	while(1){
		imu.update();
		angle = (R2D(imu.euler[1]) + 90.0);
		dt = imu.get_update_period();
		usleep(dt);

		cout << "Angle: " << fmod(angle,360.0) << endl;
	}

     return 0;
}
