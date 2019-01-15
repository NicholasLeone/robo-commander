#include <iostream>
#include "base/definitions.h"
#include <pigpiod_if2.h>
#include "bno055.h"

using namespace std;

int main(int argc, char *argv[]){

	float angle;
	int dt;
	int imu_status[3];
	int imu_revision[5];
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
		else
			printf("[SUCCESS] BNO-055 Initialized \r\n\r\n");
		// Check Status
		int ret = imu.get_system_status(&imu_status[0]);

		printf("===========    BNO-055 Status Response    =================\r\n");
	     printf("BNO-055 Status:\r\n");
	     printf("\tSystem Status: %d (%#x)\r\n", imu_status[0], imu_status[0]);
	     printf("\tSelf-Test Result: %d (%#x)\r\n", imu_status[1], imu_status[1]);
	     printf("\tSystem Error Status: %d (%#x)\r\n", imu_status[2], imu_status[2]);

		int ret2 = imu.get_revision(&imu_revision[0]);
		printf("===========    BNO-055 Software Revision Response    =================\r\n");
	     printf("BNO-055 Software Revision:\r\n");
	     printf("\tSoftware Version: %d (%#x)\r\n", imu_revision[0], imu_revision[0]);
	     printf("\tBootloader Version: %d (%#x)\r\n", imu_revision[1], imu_revision[1]);
	     printf("\tAccelerometer ID: %d (%#x)\r\n", imu_revision[2], imu_revision[2]);
	     printf("\tGyroscope ID: %d (%#x)\r\n", imu_revision[3], imu_revision[3]);
	     printf("\tMagnetometer ID: %d (%#x)\r\n", imu_revision[4], imu_revision[4]);

		printf("===========    BNO-055 Reading Euler   =================\r\n");
		uint16_t euler[3];
		imu.read_vector(0x1A & 0xFF, &euler[0]);
		float e1 = (float) euler[0] / 16.0;
		float e2 = (float) euler[1] / 16.0;
		float e3 = (float) euler[2] / 16.0;
		printf(" Euler Angles: %f, %f, %f\r\n", e1, e2 , e3);
		imu.update();
	}

     return 0;
}
