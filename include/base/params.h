#ifndef PARAMS_H_
#define PARAMS_H_

using namespace std;

/** SECTION:

     ACTUATOR PARAMETERS

*/

/** SECTION:

     SENSORS PARAMETERS

*/


typedef struct SENSOR_PARAMS{
     // Device Address
     int add;
     // Component Readings
     float x;
     float y;
     float z;
     // Component Offsets
     float x_off;
     float y_off;
     float z_off;
     // Full-Scale Range
     int fs;
     // Sensitivity
     float sens;
} SENSOR_PARAMS;


/** SECTION:

     CONTROLLERS PARAMETERS

*/



/** SECTION:

     MESSAGES TYPES

*/

typedef struct Sim_Msg_Ack{
	int32_t sequence;
}Sim_Msg_Ack;

typedef struct Sim_Msg_SetPort{
	int32_t simulatorDataType;
	int32_t port;
}Sim_Msg_SetPort;

typedef struct Sim_Msg_SimulatorControl{
	int32_t simulatorControlCode;
}Sim_Msg_SimulatorControl;

typedef void SimulatorMessage;

// typedef enum SimulatorMessageType{
// 	SIMULATOR_MESSAGE_SET_PORT = 0,
// 	SIMULATOR_MESSAGE_CONTROL = 1,
// 	SIMULATOR_MESSAGE_DATA_GPS = 2,
// 	SIMULATOR_MESSAGE_DATA_GYRO = 3,
// 	SIMULATOR_MESSAGE_DATA_ACCELERATION = 4,
// 	SIMULATOR_MESSAGE_DATA_RCCOMMANDS = 5,
// 	SIMULATOR_MESSAGE_DATA_LIDAR = 6,
// 	SIMULATOR_MESSAGE_DATA_ORIENTATION = 7,
// 	SIMULATOR_MESSAGE_DATA_ENCODER = 8,
// 	SIMULATOR_MESSAGE_DATA_FRONT_CAM = 9,
// 	SIMULATOR_MESSAGE_DATA_IMU = 9,
// }SimulatorMessageType;

typedef enum SimulatorControlCode{
	SIMULATOR_CONTROL_STOP = 0,
	SIMULATOR_CONTROL_START = 1,
	SIMULATOR_CONTROL_PAUSE = 2,
	SIMULATOR_CONTROL_RESUME = 3,
}SimulatorControlCode;

typedef struct Sim_Msg_Data_Header{
	int32_t data_type;
	int32_t component_id;
}Sim_Msg_Data_Header;

typedef enum SimulatorMessageType{
	SIMULATOR_MESSAGE_SET_PORT        = 0,
	SIMULATOR_MESSAGE_CONTROL         = 1,
	SIMULATOR_MESSAGE_SENSOR_DATA     = 2,
	SIMULATOR_MESSAGE_DATA_RCCOMMANDS = 3,
}SimulatorMessageType;

typedef enum SimulatorDataType{
     SIMULATOR_DATA_NONE          = 0,
     SIMULATOR_DATA_IMU           = 1,
     SIMULATOR_DATA_ENCODER       = 2,
     SIMULATOR_DATA_GPS           = 3,
     SIMULATOR_DATA_RGB_CAMERA    = 4,
     SIMULATOR_DATA_RGBD_CAMERA   = 5,
     SIMULATOR_DATA_LIDAR         = 6,
     SIMULATOR_DATA_3D_LIDAR      = 7,
     SIMULATOR_DATA_3D_CAMERA     = 8,
     SIMULATOR_DATA_ODOMETRY      = 9,
}SimulatorDataType;

typedef enum SimulatorMeasurementType{
     SIMULATOR_MEASUREMENT_SINGLE    = 0,
     SIMULATOR_MEASUREMENT_MULTIPLE  = 1,
}SimulatorMeasurementType;

/** SECTION:

     DATA MESSAGES TYPES

*/

typedef struct XYZ_BASE{
     float x;
     float y;
     float z;
} XYZ_BASE;

typedef struct ORIENTATION_QUATERNION_BASE{
     float x;
     float y;
     float z;
     float w;
} ORIENTATION_QUATERNION_BASE;

typedef struct ORIENTATION_EULER_BASE{
     float roll;
     float pitch;
     float yaw;
} ORIENTATION_EULER_BASE;

/** SECTION:
     EXTRAPOLATED INFORMATION
*/
typedef struct XYZ_DATA{
     float x;
     float y;
     float z;
     XYZ_BASE covariance;
     XYZ_BASE bias;
     XYZ_BASE offset;
} XYZ_DATA;

typedef struct ORIENTATION_DATA{
     float x;
     float y;
     float z;
     float w;
     ORIENTATION_EULER_BASE covariance;
     ORIENTATION_QUATERNION_BASE bias;
} ORIENTATION_DATA;

/** SECTION:
     SENSOR DATA TYPES
*/
typedef struct Sim_IMUData{
     XYZ_DATA accel;
     XYZ_DATA gyro;
     ORIENTATION_DATA orientation;
}Sim_IMUData;

typedef struct Sim_EncoderData{
     int pulses;
     float speed;
}Sim_EncoderData;

typedef struct Sim_OdometryData{
     XYZ_DATA position;
     ORIENTATION_DATA orientation;
}Sim_OdometryData;

typedef struct Sim_GPSData{
	float time;
	float latitude;
	float longitude;
	float altitude;
	XYZ_BASE velocity;
	XYZ_BASE covariance;
	uint8_t covariance_type;
	uint16_t service;
	int8_t status;
}Sim_GPSData;

typedef struct Sim_LidarData{
     int32_t angle_min;
     int32_t angle_max;
     int32_t dAngle;
     int32_t scan_time;
     int32_t dTime;
     int32_t range_min;
     int32_t range_max;
     int32_t ranges[];
     int32_t intensities[];
}Sim_LidarData;

typedef struct XYZ_BASE_INT{
	int32_t x;
	int32_t y;
	int32_t z;
} XYZ_BASE_INT;

typedef struct ORIENTATION_QUATERNION_BASE_INT{
	int32_t x;
	int32_t y;
	int32_t z;
	int32_t w;
} ORIENTATION_QUATERNION_BASE_INT;

typedef struct ORIENTATION_EULER_BASE_INT{
	int32_t roll;
	int32_t pitch;
	int32_t yaw;
} ORIENTATION_EULER_BASE_INT;

typedef struct XYZ_DATA_INT{
	int32_t x;
	int32_t y;
	int32_t z;
	XYZ_BASE covariance;
	XYZ_BASE bias;
	XYZ_BASE offset;
} XYZ_DATA_INT;

typedef struct Sim_Msg_OrientationData{
	int32_t x;
	int32_t y;
	int32_t z;
	int32_t w;
	ORIENTATION_EULER_BASE_INT covariance;
	ORIENTATION_QUATERNION_BASE_INT bias;
} Sim_Msg_OrientationData;

typedef struct Sim_Msg_IMUData{
	XYZ_DATA_INT accel;
	XYZ_DATA_INT gyro;
	Sim_Msg_OrientationData orientation;
}Sim_Msg_IMUData;

typedef struct Sim_Msg_GPSData{
	int32_t time;
	int32_t latitude;
	int32_t longitude;
	int32_t altitude;
	XYZ_BASE_INT velocity;
	XYZ_BASE_INT covariance;
	uint8_t covariance_type;
	uint16_t service;
	int8_t status;
}Sim_Msg_GPSData;

typedef struct Sim_Msg_GyroData{
	int32_t x;
	int32_t y;
	int32_t z;
}Sim_Msg_GyroData;

typedef struct Sim_Msg_AccelerationData{
	int32_t x;
	int32_t y;
	int32_t z;
}Sim_Msg_AccelerationData;


typedef struct Sim_Msg_MotionCommands{
	int32_t normalized_speed;
	int32_t padding;
	int32_t normalized_yaw_rate;
}Sim_Msg_MotionCommands;

typedef struct Sim_Msg_LidarData{
	int32_t angle_min;
	int32_t angle_max;
	int32_t dAngle;
	int32_t scan_time;
	int32_t dTime;
	int32_t range_min;
	int32_t range_max;
	int32_t ranges[1081];
	int32_t intensities[1081];
}Sim_Msg_LidarData;

typedef struct Sim_Msg_EncoderData{
	int32_t pulses;
	int32_t speed;
}Sim_Msg_EncoderData;

#endif // PARAMS_H_
