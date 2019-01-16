#ifndef BNO055_H_
#define BNO055_H_

#include "base/params.h"
#include "base/peripherals.h"
#include "comms/uart.h"

// Power Modes
typedef enum{
	// Power modes
	POWER_MODE_NORMAL                    = 0X00,
	POWER_MODE_LOWPOWER                  = 0X01,
	POWER_MODE_SUSPEND                   = 0X02
}BNO055PwrMode;

// Operation mode settings
typedef enum{
	OPERATION_MODE_CONFIG                = 0X00,
	OPERATION_MODE_ACCONLY               = 0X01,
	OPERATION_MODE_MAGONLY               = 0X02,
	OPERATION_MODE_GYRONLY               = 0X03,
	OPERATION_MODE_ACCMAG                = 0X04,
	OPERATION_MODE_ACCGYRO               = 0X05,
	OPERATION_MODE_MAGGYRO               = 0X06,
	OPERATION_MODE_AMG                   = 0X07,
	OPERATION_MODE_IMUPLUS               = 0X08,
	OPERATION_MODE_COMPASS               = 0X09,
	OPERATION_MODE_M4G                   = 0X0A,
	OPERATION_MODE_NDOF_FMC_OFF          = 0X0B,
	OPERATION_MODE_NDOF                  = 0X0C
}BNO055OpMode;

typedef enum{
	// I2C addresses
	BNO055_ADDRESS_A                     = 0x28,
	BNO055_ADDRESS_B                     = 0x29,
	BNO055_ID                            = 0xA0,

	// Page id register definition
	BNO055_PAGE_ID_ADDR                  = 0X07,

	// PAGE0 REGISTER DEFINITION START
	BNO055_CHIP_ID_ADDR                  = 0x00,
	BNO055_ACCEL_REV_ID_ADDR             = 0x01,
	BNO055_MAG_REV_ID_ADDR               = 0x02,
	BNO055_GYRO_REV_ID_ADDR              = 0x03,
	BNO055_SW_REV_ID_LSB_ADDR            = 0x04,
	BNO055_SW_REV_ID_MSB_ADDR            = 0x05,
	BNO055_BL_REV_ID_ADDR                = 0X06,

	// Accel data register
	BNO055_ACCEL_DATA_X_LSB_ADDR         = 0X08,
	BNO055_ACCEL_DATA_X_MSB_ADDR         = 0X09,
	BNO055_ACCEL_DATA_Y_LSB_ADDR         = 0X0A,
	BNO055_ACCEL_DATA_Y_MSB_ADDR         = 0X0B,
	BNO055_ACCEL_DATA_Z_LSB_ADDR         = 0X0C,
	BNO055_ACCEL_DATA_Z_MSB_ADDR         = 0X0D,

	// Mag data register
	BNO055_MAG_DATA_X_LSB_ADDR           = 0X0E,
	BNO055_MAG_DATA_X_MSB_ADDR           = 0X0F,
	BNO055_MAG_DATA_Y_LSB_ADDR           = 0X10,
	BNO055_MAG_DATA_Y_MSB_ADDR           = 0X11,
	BNO055_MAG_DATA_Z_LSB_ADDR           = 0X12,
	BNO055_MAG_DATA_Z_MSB_ADDR           = 0X13,

	// Gyro data registers
	BNO055_GYRO_DATA_X_LSB_ADDR          = 0X14,
	BNO055_GYRO_DATA_X_MSB_ADDR          = 0X15,
	BNO055_GYRO_DATA_Y_LSB_ADDR          = 0X16,
	BNO055_GYRO_DATA_Y_MSB_ADDR          = 0X17,
	BNO055_GYRO_DATA_Z_LSB_ADDR          = 0X18,
	BNO055_GYRO_DATA_Z_MSB_ADDR          = 0X19,

	// Euler data registers
	BNO055_EULER_H_LSB_ADDR              = 0X1A,
	BNO055_EULER_H_MSB_ADDR              = 0X1B,
	BNO055_EULER_R_LSB_ADDR              = 0X1C,
	BNO055_EULER_R_MSB_ADDR              = 0X1D,
	BNO055_EULER_P_LSB_ADDR              = 0X1E,
	BNO055_EULER_P_MSB_ADDR              = 0X1F,

	// Quaternion data registers
	BNO055_QUATERNION_DATA_W_LSB_ADDR    = 0X20,
	BNO055_QUATERNION_DATA_W_MSB_ADDR    = 0X21,
	BNO055_QUATERNION_DATA_X_LSB_ADDR    = 0X22,
	BNO055_QUATERNION_DATA_X_MSB_ADDR    = 0X23,
	BNO055_QUATERNION_DATA_Y_LSB_ADDR    = 0X24,
	BNO055_QUATERNION_DATA_Y_MSB_ADDR    = 0X25,
	BNO055_QUATERNION_DATA_Z_LSB_ADDR    = 0X26,
	BNO055_QUATERNION_DATA_Z_MSB_ADDR    = 0X27,

	// Linear acceleration data registers
	BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR  = 0X28,
	BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR  = 0X29,
	BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR  = 0X2A,
	BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR  = 0X2B,
	BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR  = 0X2C,
	BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR  = 0X2D,

	// Gravity data registers
	BNO055_GRAVITY_DATA_X_LSB_ADDR       = 0X2E,
	BNO055_GRAVITY_DATA_X_MSB_ADDR       = 0X2F,
	BNO055_GRAVITY_DATA_Y_LSB_ADDR       = 0X30,
	BNO055_GRAVITY_DATA_Y_MSB_ADDR       = 0X31,
	BNO055_GRAVITY_DATA_Z_LSB_ADDR       = 0X32,
	BNO055_GRAVITY_DATA_Z_MSB_ADDR       = 0X33,

	// Temperature data register
	BNO055_TEMP_ADDR                     = 0X34,

	// Status registers
	BNO055_CALIB_STAT_ADDR               = 0X35,
	BNO055_SELFTEST_RESULT_ADDR          = 0X36,
	BNO055_INTR_STAT_ADDR                = 0X37,

	BNO055_SYS_CLK_STAT_ADDR             = 0X38,
	BNO055_SYS_STAT_ADDR                 = 0X39,
	BNO055_SYS_ERR_ADDR                  = 0X3A,

	// Unit selection register
	BNO055_UNIT_SEL_ADDR                 = 0X3B,
	BNO055_DATA_SELECT_ADDR              = 0X3C,

	// Mode registers
	BNO055_OPR_MODE_ADDR                 = 0X3D,
	BNO055_PWR_MODE_ADDR                 = 0X3E,

	BNO055_SYS_TRIGGER_ADDR              = 0X3F,
	BNO055_TEMP_SOURCE_ADDR              = 0X40,

	// Axis remap registers
	BNO055_AXIS_MAP_CONFIG_ADDR          = 0X41,
	BNO055_AXIS_MAP_SIGN_ADDR            = 0X42,

	// Axis remap values
	AXIS_REMAP_X                         = 0x00,
	AXIS_REMAP_Y                         = 0x01,
	AXIS_REMAP_Z                         = 0x02,
	AXIS_REMAP_POSITIVE                  = 0x00,
	AXIS_REMAP_NEGATIVE                  = 0x01,

	// SIC registers
	BNO055_SIC_MATRIX_0_LSB_ADDR         = 0X43,
	BNO055_SIC_MATRIX_0_MSB_ADDR         = 0X44,
	BNO055_SIC_MATRIX_1_LSB_ADDR         = 0X45,
	BNO055_SIC_MATRIX_1_MSB_ADDR         = 0X46,
	BNO055_SIC_MATRIX_2_LSB_ADDR         = 0X47,
	BNO055_SIC_MATRIX_2_MSB_ADDR         = 0X48,
	BNO055_SIC_MATRIX_3_LSB_ADDR         = 0X49,
	BNO055_SIC_MATRIX_3_MSB_ADDR         = 0X4A,
	BNO055_SIC_MATRIX_4_LSB_ADDR         = 0X4B,
	BNO055_SIC_MATRIX_4_MSB_ADDR         = 0X4C,
	BNO055_SIC_MATRIX_5_LSB_ADDR         = 0X4D,
	BNO055_SIC_MATRIX_5_MSB_ADDR         = 0X4E,
	BNO055_SIC_MATRIX_6_LSB_ADDR         = 0X4F,
	BNO055_SIC_MATRIX_6_MSB_ADDR         = 0X50,
	BNO055_SIC_MATRIX_7_LSB_ADDR         = 0X51,
	BNO055_SIC_MATRIX_7_MSB_ADDR         = 0X52,
	BNO055_SIC_MATRIX_8_LSB_ADDR         = 0X53,
	BNO055_SIC_MATRIX_8_MSB_ADDR         = 0X54,

	// Accelerometer Offset registers
	ACCEL_OFFSET_X_LSB_ADDR              = 0X55,
	ACCEL_OFFSET_X_MSB_ADDR              = 0X56,
	ACCEL_OFFSET_Y_LSB_ADDR              = 0X57,
	ACCEL_OFFSET_Y_MSB_ADDR              = 0X58,
	ACCEL_OFFSET_Z_LSB_ADDR              = 0X59,
	ACCEL_OFFSET_Z_MSB_ADDR              = 0X5A,

	// Magnetometer Offset registers
	MAG_OFFSET_X_LSB_ADDR                = 0X5B,
	MAG_OFFSET_X_MSB_ADDR                = 0X5C,
	MAG_OFFSET_Y_LSB_ADDR                = 0X5D,
	MAG_OFFSET_Y_MSB_ADDR                = 0X5E,
	MAG_OFFSET_Z_LSB_ADDR                = 0X5F,
	MAG_OFFSET_Z_MSB_ADDR                = 0X60,

	// Gyroscope Offset register s
	GYRO_OFFSET_X_LSB_ADDR               = 0X61,
	GYRO_OFFSET_X_MSB_ADDR               = 0X62,
	GYRO_OFFSET_Y_LSB_ADDR               = 0X63,
	GYRO_OFFSET_Y_MSB_ADDR               = 0X64,
	GYRO_OFFSET_Z_LSB_ADDR               = 0X65,
	GYRO_OFFSET_Z_MSB_ADDR               = 0X66,

	// Radius registers
	ACCEL_RADIUS_LSB_ADDR                = 0X67,
	ACCEL_RADIUS_MSB_ADDR                = 0X68,
	MAG_RADIUS_LSB_ADDR                  = 0X69,
	MAG_RADIUS_MSB_ADDR                  = 0X6A
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
	UartDev* _sd;
	int _baud;
	char _uart_buffer[4096];
	uint8_t _mode;

	int max_retry_attempts = 2;
	float timeout;
     bool _initialized = false;
	bool _verbose = false;
     ImuData _readings;

	float Caccel_fct = 1000.0;
	float Cmag_fct = 16.0;
	float Cgyro_fct = 900.0;

	char* _pi_read(int num_bytes, bool verbose = true);
	char* _uart_send(char* cmds, bool ack = true, bool verbose = true, int max_trys = 5);

	int _read(int num_bytes, char* data);
	int _send(char* cmds, int length, char* data, bool ack = true, bool verbose = true, int max_trys = 5);

	int _write_bytes(uint8_t _address, uint8_t* bytes, bool ack = true);
	int _write_byte(uint8_t _address, uint8_t byte, bool ack = true);

	int _read_bytes(uint8_t _address, int length, uint8_t* recv_data);
	int _read_byte(uint8_t _address, uint8_t* recv_data);
	int8_t _read_signed_byte(uint8_t _address);

	void _config_mode();
	void _operation_mode();

	int read_vector(uint8_t _address, int16_t* data, int count = 3);
	void flush();
	int available();
public:

     // FUNCTIONS
	BNO055(std::string dev, int baud);
     ~BNO055();

	int begin(uint8_t mode = OPERATION_MODE_NDOF);

	void update(bool verbose = false);

	/** ==================================================================
	*						    SETTERS
	* ==================================================================== */
	int set_mode(uint8_t mode);
	void set_external_crystal(bool use_external_crystal);

	/**
	*	@desc: Set the sensor's calibration data using a list of 22 bytes that
   	* represent the sensor offsets and calibration data.  This data should be
   	* a value that was previously retrieved with get_calibration (and then
   	* perhaps persisted to disk or other location until needed again).
	*/
	int set_calibration(uint8_t* data);

	/**
	*	@desc: Set axis remap for each axis.  The x, y, z parameter values should
   	* be set to one of AXIS_REMAP_X, AXIS_REMAP_Y, or AXIS_REMAP_Z and will
   	* change the BNO's axis to represent another axis.  Note that two axises
   	* cannot be mapped to the same axis, so the x, y, z params should be a
   	* unique combination of AXIS_REMAP_X, AXIS_REMAP_Y, AXIS_REMAP_Z values.
   	* The x_sign, y_sign, z_sign values represent if the axis should be positive
   	* or negative (inverted).
	*/
	int set_axis_remap(int x, int y, int z, int xsign, int ysign, int zsign);


	/** ==================================================================
	*						    Getters
	* ==================================================================== */
	/**
	* @desc: Returns the following revision information about the BNO055 chip.
	*
	*    @return[0]: Software revision      - 2 bytes (MSB + LSB)
	*    @return[1]: Bootloader version     - 1 byte
	*    @return[2]: Accelerometer ID       - 1 byte
	*    @return[3]: Gyro ID                - 1 byte
	*    @return[4]: Magnetometer ID        - 1 byte
	*/
	void get_revision(int* revision_data);

	/**
	* @desc: Return a tuple with status information.  Three values will be returned:
	*
	*    @output - System status register value with the following meaning:
	*         0 = Idle
	*         1 = System Error
	*         2 = Initializing Peripherals
	*         3 = System Initialization
	*         4 = Executing Self-Test
	*         5 = Sensor fusion algorithm running
	*         6 = System running without fusion algorithms
	*    @output - Self test result register value with the following meaning:
	*         Bit value: 1 = test passed, 0 = test failed
	*         Bit 0 = Accelerometer self test
	*         Bit 1 = Magnetometer self test
	*         Bit 2 = Gyroscope self test
	*         Bit 3 = MCU self test
	*         Value of 0x0F = all good!
	*    @output - System error register value with the following meaning:
	*         0 = No error
	*         1 = Peripheral initialization error
	*         2 = System initialization error
	*         3 = Self test result failed
	*         4 = Register map value out of range
	*         5 = Register map address out of range
	*         6 = Register map write error
	*         7 = BNO low power mode not available for selected operation mode
	*         8 = Accelerometer power mode not available
	*         9 = Fusion algorithm configuration error
	*         10 = Sensor configuration error
	*
	* If run_self_test is passed in as False then no self test is performed and
	* None will be returned for the self test result.  Note that running a
	* self test requires going into config mode which will stop the fusion
	* engine from running.
	*/
	void get_system_status(int* status, bool run_self_test = true);

	/**
	* @desc: Read the calibration status of the sensors
	*
	*    @return[0]: System		 - 3 (fully calibrated), 0 (not calibrated)
	*    @return[1]: Gyroscope	 - 3 (fully calibrated), 0 (not calibrated)
	*    @return[2]: Accelerometer - 3 (fully calibrated), 0 (not calibrated)
	*    @return[3]: Magnetometer	 - 3 (fully calibrated), 0 (not calibrated)
	*/
	void get_calibration_status(int* status);

	/**
	*	@desc: Return the sensor's calibration data (made up of 22 bytes).
	* Can be saved and reloaded with the set_calibration function for quick
	* calibrate from a previously calculated set of calibration data.
	*
	*	@output data[0-2]:	Accelerometer Offset (X, Y, Z)
	*	@output data[3-5]:	Magnetometer Offset (X, Y, Z)
	*	@output data[6-8]:	Gyroscope Offset (X, Y, Z)
	*	@output data[9]:	Radius of the Accelerometer
	*	@output data[10]:	Radius of the Magnetometer
	*/
	void get_calibration(float* data);

	/**
	*	@desc: Return 6 axis remap register values
   	*
	*	@output data[0] - X axis remap (a value of AXIS_REMAP_X, AXIS_REMAP_Y,
	*				  or AXIS_REMAP_Z, which indicates that the physical
	*				  X axis of the chip is remapped to a different axis)
	*	@output data[1] - Y axis remap (see above)
	*	@output data[2] - Z axis remap (see above)
	*	@output data[3] - X axis sign (a value of AXIS_REMAP_POSITIVE or
	*				  AXIS_REMAP_NEGATIVE which indicates if the X axis
	*				  values should be positive/normal or negative/inverted.
	*				  The default is positive.)
	*	@output data[4] - Y axis sign (see above)
	*	@output data[5] - Z axis sign (see above)
   	*
	* 	@note: by default the axis orientation of the BNO-055 chip looks like
   	* the following (taken from section 3.4, page 24 of the datasheet). Notice
   	* the dot in the corner that corresponds to the dot on the BNO chip:
	*			  | Z axis
	*			  |
	*			  |   / X axis
	*		   ____|__/____
	* Y axis   / *   | /    /|
	*________ /______|/    //
	*	    /___________ //
	*	   |____________|/
	*/
	void get_axis_remap(int* data);

	/**
	*	@desc: Return the current orientation in degrees (roll, pitch, yaw)
	*/
	void get_euler(float* data, bool verbose = false);

	/**
	*	@desc: Return the current magnetometer reading in micro-Teslas.
	*/
	void get_magnetometer(float* data, bool verbose = false);

	/**
	*	@desc: Return the current gyroscope (angular velocity) in rad/sec.
	*/
	void get_gyroscope(float* data, bool verbose = false);

	/**
	*	@desc: Return the current accelerometer reading in m/s^2.
	*/
	void get_accelerometer(float* data, bool verbose = false);

	/**
	*	@desc: Return the current linear acceleration (i.e. acceleration
	* from movement, not from gravity) in m/s^2.
	*/
	void get_linear_acceleration(float* data, bool verbose = false);

	/**
	*	@desc: Return the current gravity acceleration in m/s^2.
	*/
	void get_gravity(float* data, bool verbose = false);

	/**
	*	@desc: Return the current orientation as X, Y, Z, W quaternions.
	*/
	void get_quaternions(float* data, bool verbose = false);

	/**
	*	@desc: Return the current temperature in Celsius.
	*/
	float get_tempurature(bool verbose = false);

};

#endif // BNO055_H_
