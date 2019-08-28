#include <pigpiod_if2.h>
#include "sensors/mpu9250.h"
#include "communication/serial.h"

// #define DEBUG_I2C_READ
// #define TEST_CONFIG_SETTINGS
// #define TEST_CALIBRATION
#define TEST_IMU_READINGS
// #define TEST_ACCEL_READINGS
// #define TEST_GYRO_READINGS
// #define TEST_EULER_READINGS
#define TEST_MAG_READINGS

using namespace std;

char* host = NULL; // Use NULL when working on localhost
char* port = NULL; // Use NULL when working on localhost

int flag_exit = 0;

void my_handler(int s){
	printf("Caught signal %d\n",s);
	flag_exit = 1;
}

int main(int argc, char *argv[]){

	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	INTERFACE_PARAMS iface;
	IMU_CONFIG config, tmpConf;

	iface.cMeth = "serial";
	iface.host = host;
	iface.port = port;
	iface.i2c.add = IMU_ADD;
	iface.i2c.bus = IMU_I2C_BUS;
	iface.serParams.add = "/dev/ttyACM0";
	iface.serParams.baud = 115200;

	config.fs_gyro = 1000;
	config.fs_accel = 8;
	config.fs_mag = 1;

     MPU9250 imu(iface, config);



#ifdef TEST_CONFIG_SETTINGS
	tmpConf = imu.getConfig();
	int err = imu.setConfig(config);
	tmpConf = imu.getConfig();
#endif

#ifdef TEST_IMU_READINGS
	while(!flag_exit){

#ifdef TEST_ACCEL_READINGS
		imu.updateAccel();
#endif

#ifdef TEST_GYRO_READINGS
		imu.updateGyro();
#endif

#ifdef TEST_EULER_READINGS
	vector<float> eulers = imu.getEuler();
#endif

#ifdef TEST_MAG_READINGS
	imu.updateMag();
#endif


	}
#endif

     return 0;
}


/** TO COMPILE:

     g++ -w test_read.cpp mpu9250.cpp serial.cpp -o TestImuI2c -lpigpiod_if2 -Wall -pthread -lboost_system -lboost_thread -lserial

*/
