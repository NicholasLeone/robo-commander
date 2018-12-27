#include <iostream>
#include "base/definitions.h"
#include <pigpiod_if2.h>
#include "bno055.h"

using namespace std;

int main(int argc, char *argv[]){

	float angle;
	int dt;

	string ttyDev = "/dev/ttyS0";
	int baud = 115200;

	int pi = pigpio_start(NULL, NULL);
	if(pi >= 0){
		BNO055 imu(pi, ttyDev, baud);
		int err = imu.begin();
		printf("[ERROR] BNO055::begin] ---- %d.\r\n", err);
	}

     return 0;
}
