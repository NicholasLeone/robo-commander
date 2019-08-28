#ifndef MPU9250_H_
#define MPU9250_H_

#include "base/params.h"
#include "base/peripherals.h"
#include "communication/serial.h"

#define DEBUG_INTERMEDIATE_IMU
#define MAX_CAL_STEPS 1000
#define USE_OFFSETS
// #define DEBUG_CONFIG_INIT

#define IMU_ADD               0x68 // i2c Address of Razor IMU
#define IMU_I2C_BUS           0x01
#define REG_SELF_TEST         0x00 // TODO. gyro-3 bytes then accel 3 bytes
#define REG_GYRO_OFFSET       0x13 // TODO. 6 bytes- H then L
#define REG_ACCEL_OFFSET      0x77 // TODO. 6 bytes- H then L
#define REG_CONFIG            0x1A // 4 Bytes after
#define REG_GYRO_CONFIG       0x1B
#define REG_ACCEL_CONFIG      0x1C
#define REG_ACCEL2_CONFIG     0x1D
#define REG_FIFO_EN           0x23 // TODO
#define REG_ACCEL_DATA        0x3B
#define REG_GYRO_DATA         0x43
#define REG_TEMP_DATA         0x41
#define REG_MAG_ADD           0x48
#define REG_PWR_MGMT_1        0x6B

using namespace std;

typedef struct IMU_CONFIG{

     // Full-Scale Ranges
     int fs_gyro;
     int fs_accel;
     int fs_mag;

     // TODO: output rates, fifo-related settings, dmp-related settings, etc.

} IMU_CONFIG;

typedef struct velocity_t {
     float x;
     float y;
     float z;
} velocity_t;

typedef struct position_t {
     float x;
     float y;
     float z;
} position_t;

typedef struct attitude_t{

     // Quaternions
     float qx;
     float qy;
     float qz;
     float qw;

     // Euler Angles
     float pitch;
     float roll;
     float yaw;

     // Compass Heading
     float heading;

} attitude_t;

/** END SEPERATE */


class MPU9250 {

private:

     // DERIVED Attitude
     attitude_t att;

     vector<float> updateSerial();

public:

     int _pi;
     int _com;
     SerialDev* _ser;

     // Parameters
     INTERFACE_PARAMS iface;       // Handles used for pigpiod library
     IMU_CONFIG config;            // MPU-9250 Configuration Settings

     SENSOR_PARAMS accel;          // ACCELEROMETER
     SENSOR_PARAMS gyro;           // GYROSCOPE
     SENSOR_PARAMS mag;            // MAGNETOMETER
     SENSOR_PARAMS temperature;    // Temperature

     // FUNCTIONS
	MPU9250(INTERFACE_PARAMS in_iface, IMU_CONFIG in_config);
     ~MPU9250();

     // Accelerometer
	void updateAccel();
     float getAccelSens();
     void calibrateAccel();

     // Gyroscope
	void updateGyro();
     float getGyroSens();
     void calibrateGyro();

     /** TODO: MAGNETOMETER */
     void updateMag();   //TODO: Handle i2c
     // void calibrateMag();
     float getMagSens();

     // Derived Measurements
     float getEulerX();
     float getEulerY();
     float getEulerZ();
     vector<float> getEuler();

     // TODO: Test and verify actual changing of stored settings
     IMU_CONFIG getConfig();
     int setConfig(IMU_CONFIG config);

};

#endif // MPU9250_H_

/** TODO: Try reading from FIFO instead of calculating everything manually for CPU offloading

// dmpUpdateFifo -- Reads from the top of the FIFO and fills accelerometer, gyroscope,
// quaternion, and time public variables (depending on how the DMP is configured).
// Should be called whenever an MPU interrupt is detected
// Output: INV_SUCCESS (0) on success, otherwise error
inv_error_t dmpUpdateFifo(void);

inv_error_t MPU9250_DMP::dmpUpdateFifo(void)
{
	short gyro[3];
	short accel[3];
	long quat[4];
	unsigned long timestamp;
	short sensors;
	unsigned char more;

	if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more)
		   != INV_SUCCESS)
    {
	   return INV_ERROR;
    }

	if (sensors & INV_XYZ_ACCEL)
	{
		ax = accel[X_AXIS];
		ay = accel[Y_AXIS];
		az = accel[Z_AXIS];
	}
	if (sensors & INV_X_GYRO)
		gx = gyro[X_AXIS];
	if (sensors & INV_Y_GYRO)
		gy = gyro[Y_AXIS];
	if (sensors & INV_Z_GYRO)
		gz = gyro[Z_AXIS];
	if (sensors & INV_WXYZ_QUAT)
	{
		qw = quat[0];
		qx = quat[1];
		qy = quat[2];
		qz = quat[3];
	}

	time = timestamp;

	return INV_SUCCESS;
}


END TODO*/
