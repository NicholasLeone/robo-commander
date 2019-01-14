#include <iostream>
#include "base/definitions.h"
#include <pigpiod_if2.h>
#include "bno055.h"

using namespace std;

int main(int argc, char *argv[]){

	float angle;
	int dt;

	string ttyDev = "/dev/ttyUSB0";
	int baud = 115200;

	// string ttyDev = "/dev/pts/3";
	// int baud = 9762;

	int pi = pigpio_start(NULL, NULL);
	pi = 0;
	if(pi >= 0){
		// BNO055 imu(pi, ttyDev, baud);
		BNO055 imu(ttyDev, baud);
		int err = imu.begin();
		if(err < 0)
			printf("[ERROR] BNO055::begin] ---- %d.\r\n", err);
		// Check Status
		int* ret = imu.get_system_status();

		printf("===========    BNO-055 Status Response    =================\r\n");
	     printf("BNO-055 Status:\r\n");
	     printf("\tSystem Status: %d\r\n", ret[0]);
	     printf("\tSelf-Test Result: %d\r\n", ret[1]);
	     printf("\tSystem Error Status: %d\r\n", ret[2]);

		int* ret2 = imu.get_revision();
		printf("===========    BNO-055 Software Revision Response    =================\r\n");
	     printf("BNO-055 Software Revision:\r\n");
	     printf("\tSoftware Version: %d\r\n", ret[0]);
	     printf("\tBootloader Version: %d\r\n", ret[1]);
	     printf("\tAccelerometer ID: %d\r\n", ret[2]);
	     printf("\tGyroscope ID: %d\r\n", ret[3]);
	     printf("\tMagnetometer ID: %d\r\n", ret[4]);

		imu.update();
	}

     return 0;
}
