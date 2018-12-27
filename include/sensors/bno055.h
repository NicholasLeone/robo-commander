#ifndef BNO055_H_
#define BNO055_H_

#include "base/params.h"
#include "base/peripherals.h"
#include "comms/serial.h"

typedef enum{
	PWR_MODE_NORMAL,
	PWR_MODE_LOW,
	PWR_MODE_SUSPEND
}BNO055PwrMode;

typedef enum{
	OP_MODE_CONFIG,
	OP_MODE_ACCONLY,
	OP_MODE_MAGONLY,
	OP_MODE_GYROONLY,
	OP_MODE_ACCMAG,
	OP_MODE_ACCGYRO,
	OP_MODE_MAGGYRO,
	OP_MODE_AMG,
	OP_MODE_IMU,
	OP_MODE_COMPASS,
	OP_MODE_M4G,
	OP_MODE_NDOF_FMC_OFF,
	OP_MODE_NDOF,
}BNO055OpMode;

typedef enum{
	PAGE_ID=0x07,

	//PAGE 0
	PAGE0_CHIP_ID=0,
	PAGE0_ACC_ID,
	PAGE0_MAG_ID,
	PAGE0_GYR_ID,
	PAGE0_SW_REV_ID_LSB,
	PAGE0_SW_REV_ID_MSB,
	PAGE0_BL_Rev_ID,
	PAGE0_Page_ID,
	PAGE0_ACC_DATA_X_LSB,
	PAGE0_ACC_DATA_X_MSB,
	PAGE0_ACC_DATA_Y_LSB,
	PAGE0_ACC_DATA_Y_MSB,
	PAGE0_ACC_DATA_Z_LSB,
	PAGE0_ACC_DATA_Z_MSB,
	PAGE0_MAG_DATA_X_LSB,
	PAGE0_MAG_DATA_X_MSB,
	PAGE0_MAG_DATA_Y_LSB,
	PAGE0_MAG_DATA_Y_MSB,
	PAGE0_MAG_DATA_Z_LSB,
	PAGE0_MAG_DATA_Z_MSB,
	PAGE0_GYR_DATA_X_LSB,
	PAGE0_GYR_DATA_X_MSB,
	PAGE0_GYR_DATA_Y_LSB,
	PAGE0_GYR_DATA_Y_MSB,
	PAGE0_GYR_DATA_Z_LSB,
	PAGE0_GYR_DATA_Z_MSB,
	PAGE0_EUL_Heading_LSB,
	PAGE0_EUL_Heading_MSB,
	PAGE0_EUL_Roll_LSB,
	PAGE0_EUL_Roll_MSB,
	PAGE0_EUL_Pitch_LSB,
	PAGE0_EUL_Pitch_MSB,
	PAGE0_QUA_Data_w_LSB,
	PAGE0_QUA_Data_w_MSB,
	PAGE0_QUA_Data_x_LSB,
	PAGE0_QUA_Data_x_MSB,
	PAGE0_QUA_Data_y_LSB,
	PAGE0_QUA_Data_y_MSB,
	PAGE0_QUA_Data_z_LSB,
	PAGE0_QUA_Data_z_MSB,
	PAGE0_LIA_Data_X_LSB,
	PAGE0_LIA_Data_X_MSB,
	PAGE0_LIA_Data_Y_LSB,
	PAGE0_LIA_Data_Y_MSB,
	PAGE0_LIA_Data_Z_LSB,
	PAGE0_LIA_Data_Z_MBS,
	PAGE0_GRV_Data_X_LSB,
	PAGE0_GRV_Data_X_MSB,
	PAGE0_GRV_Data_Y_LSB,
	PAGE0_GRV_Data_Y_MSB,
	PAGE0_GRV_Data_Z_LSB,
	PAGE0_GRV_Data_Z_MSB,
	PAGE0_TEMP,
	PAGE0_CALIB_STAT,
	PAGE0_ST_RESULT,
	PAGE0_INT_STA,
	PAGE0_SYS_CLK_STATUS,
	PAGE0_SYS_STATUS,
	PAGE0_SYS_ERR,
	PAGE0_UNIT_SEL,
	PAGE0_Reserved,
	PAGE0_OPR_MODE,
	PAGE0_PWR_MODE,
	PAGE0_SYS_TRIGGER,
	PAGE0_TEMP_SOURCE,
	PAGE0_AXIS_MAP_CONFIG,
	PAGE0_AXIS_MAP_SIGN,
	//RESERVED
	PAGE0_ACC_OFFSET_X_LSB=0x55,
	PAGE0_ACC_OFFSET_X_MSB,
	PAGE0_ACC_OFFSET_Y_LSB,
	PAGE0_ACC_OFFSET_Y_MSB,
	PAGE0_ACC_OFFSET_Z_LSB,
	PAGE0_ACC_OFFSET_Z_MSB,
	PAGE0_MAG_OFFSET_X_LSB,
	PAGE0_MAG_OFFSET_X_MSB,
	PAGE0_MAG_OFFSET_Y_LSB,
	PAGE0_MAG_OFFSET_Y_MSB,
	PAGE0_MAG_OFFSET_Z_LSB,
	PAGE0_MAG_OFFSET_Z_MSB,
	PAGE0_GYR_OFFSET_X_LSB,
	PAGE0_GYR_OFFSET_X_MSB,
	PAGE0_GYR_OFFSET_Y_LSB,
	PAGE0_GYR_OFFSET_Y_MSB,
	PAGE0_GYR_OFFSET_Z_LSB,
	PAGE0_GYR_OFFSET_Z_MSB,
	PAGE0_ACC_RADIUS_LSB,
	PAGE0_ACC_RADIUS_MSB,
	PAGE0_MAG_RADIUS_LSB,
	PAGE0_MAG_RADIUS_MSB,

	PAGE1_Page_ID=0x07,
	PAGE1_ACC_Config,
	PAGE1_MAG_Config,
	PAGE1_GYR_Config_0,
	PAGE1_GYR_Config_1,
	PAGE1_ACC_Sleep_Config,
	PAGE1_GYR_Sleep_Config,
	PAGE1_Reserved,
	PAGE1_INT_MSK,
	PAGE1_INT_EN,
	PAGE1_ACC_AM_THRES,
	PAGE1_ACC_INT_SET,
	PAGE1_ACC_HG_DURATION,
	PAGE1_ACC_HG_THRES,
	PAGE1_ACC_NM_THRES,
	PAGE1_ACC_NM_SET,
	PAGE1_GYR_INT_SET,
	PAGE1_GYR_HR_X_SET,
	PAGE1_GYR_DUR_X,
	PAGE1_GYR_HR_Y_SET,
	PAGE1_GYR_DUR_Y,
	PAGE1_GYR_HR_Z_SET,
	PAGE1_GYR_DUR_Z,
	PAGE1_GYR_AM_THRES,
	PAGE1_GYR_AM_SET,
	//RESERVED
	PAGE1_UNIQE_ID_B0=0x50,
	PAGE1_UNIQE_ID_B1=0x51,
	PAGE1_UNIQE_ID_B2=0x52,
	PAGE1_UNIQE_ID_B3=0x53,
	PAGE1_UNIQE_ID_B4=0x54,
	PAGE1_UNIQE_ID_B5=0x55,
	PAGE1_UNIQE_ID_B6=0x56,
	PAGE1_UNIQE_ID_B7=0x57,
	PAGE1_UNIQE_ID_B8=0x58,
	PAGE1_UNIQE_ID_B9=0x59,
	PAGE1_UNIQE_ID_BA=0x5A,
	PAGE1_UNIQE_ID_BB=0x5B,
	PAGE1_UNIQE_ID_BC=0x5C,
	PAGE1_UNIQE_ID_BD=0x5D,
	PAGE1_UNIQE_ID_BE=0x5E,
	PAGE1_UNIQE_ID_BF=0x5F,
	//RESERVED
}BNO055Register;

typedef struct{
	union{
		struct{
			int16_t AccelerationDataX;//0 1
			int16_t AccelerationDataY;//2 3
			int16_t AccelerationDataZ;//4 5
			int16_t MagnetometerDataX;//6 7
			int16_t MagnetometerDataY;//8 9
			int16_t MagnetometerDataZ;//10 11
			int16_t GyroscopeDataX;//12 13
			int16_t GyroscopeDataY;//14 15
			int16_t GyroscopeDataZ;//16 17
			int16_t HeadingData;//18 19
			int16_t RollData;//20 21
			int16_t PitchData;//22 23
			int16_t QuaternionwData;// 24 25
			int16_t QuaternionxData;// 26 27
			int16_t QuaternionyData;// 28 29
			int16_t QuaternionzData;// 30 31
			int16_t LinearAccelerationDataX;//32 33
			int16_t LinearAccelerationDataY;//34 35
			int16_t LinearAccelerationDataZ;//36 37
			int16_t GravityVectorDataX;//38 39
			int16_t GravityVectorDataY;//40 41
			int16_t GravityVectorDataZ;//42 43
		}imu;
		uint16_t nthos_array[22];
	};
	int8_t temperature;
	uint8_t calibration_status;
}ImuData;


using namespace std;

class BNO055 {

private:
     int _pi;
	int _handle;
	int _baud;

	int max_retry_attempts = 2;
	float timeout;
     bool _initialized = false;
     ImuData _readings;

     int _write(uint8_t* bytes, int length, bool ack = true, int max_trys = 5);
	int _imu_write_bytes(BNO055Register reg, uint8_t* bytes, int length);
     int _read(uint8_t* buf, int length);

     float Caccel_fct = 1000.0;
     float Cmag_fct = 16.0;
     float Cgyro_fct = 900.0;

public:

     // FUNCTIONS
	BNO055(int pi, std::string dev, int baud);
     ~BNO055();

     int begin(BNO055OpMode mode = OP_MODE_NDOF);

     int _imu_write_byte(BNO055Register reg, uint8_t byte);
     int read(BNO055Register reg, uint8_t *data, int length);

     void flush();
     int available();
     void update();

     // // Accelerometer
     // void updateAccel();
     // float getAccelSens();
     // void calibrateAccel();
     //
     // // Gyroscope
     // void updateGyro();
     // float getGyroSens();
     // void calibrateGyro();
     //
     // /** TODO: MAGNETOMETER */
     // void updateMag();   //TODO: Handle i2c
     // // void calibrateMag();
     // float getMagSens();
     //
     // // Derived Measurements
     // float getEulerX();
     // float getEulerY();
     // float getEulerZ();
     // vector<float> getEuler();
     //
     // // TODO: Test and verify actual changing of stored settings
     // IMU_CONFIG getConfig();
     // int setConfig(IMU_CONFIG config);
};

#endif // BNO055_H_
