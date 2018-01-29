#ifndef PARAMS_H_
#define PARAMS_H_

#include <armadillo>
#include <string.h>
#include "definitions.h"

/** UDP_PARAMS Needs these */
#include <netinet/in.h>
#include <netdb.h>
#include <sys/time.h>

using namespace std;
using namespace arma;

#define M_DEG2RAD (2*M_PI)/360

/** SECTION:

     ACTUATOR PARAMETERS

*/

#define SetDWORDval(arg) (uint8_t)(((uint32_t)arg)>>24),(uint8_t)(((uint32_t)arg)>>16),(uint8_t)(((uint32_t)arg)>>8),(uint8_t)arg
#define SetWORDval(arg) (uint8_t)(((uint16_t)arg)>>8),(uint8_t)arg

enum PERIPHERAL_PROTOCOL {
	I2C,
	UART,
	GPIO,
	PWM,
	SPI,
	BLUETOOTH,
	WIFI,
	BLUETOOTH_LE,
	MAVLINK2_0,
	MAVLINK3_0
};

static const int NUM_MOTOR_PINS = 2;

typedef struct PIN_CONFIG{
     int pinNum;
     int pinLevel;
} PIN_CONFIG;

typedef struct MOTOR_PARAMS{
     PIN_CONFIG direction_gpio[NUM_MOTOR_PINS];
     int direction;
     int pwm_channel;
     int pwm_val;
} MOTOR_PARAMS;

typedef struct SERVO_PARAMS{
     int channel;
     float max_angle;
     float min_angle;
     float current_angle;
     int max_pulse;
     int min_pulse;
     int zero_pulse;
     int current_pulse;
} SERVO_PARAMS;

/** SECTION:

     COMMUNICATION INTERFACES PARAMETERS

*/

typedef struct I2C_PARAMS{
     int add; // address of device
     int bus; // i2c bus of device
} I2C_PARAMS;

typedef struct SERIAL_PARAMS{
     char* add; // Address of device, e.g "/dev/ttyACM0"
     unsigned int baud; // Baud Rate
} SERIAL_PARAMS;

typedef struct INTERFACE_PARAMS{
     string cMeth;  // Method to use for Device Communication
     char* host;    //
     char* port;    //

     // Communications
     I2C_PARAMS i2c;
     SERIAL_PARAMS serParams;

     int pi;        // Raspberry Pi Device is On
     int com;       // Communication Handle of device for pigpiod library

} INTERFACE_PARAMS;

typedef struct UDP_PARAMS{
     int fd;
     int sock;
     int in_port;
     int dev_address;
     int out_port;
     char* file;
     socklen_t sock_len;
     struct sockaddr_in remAddr;
     struct sockaddr_in myAddr;
     struct hostent * hp;
     fd_set read_fds;
     fd_set write_fds;
     struct timeval tv;
} UDP_PARAMS;


/** SECTION:

     SENSORS PARAMETERS

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

typedef struct EKF_PARAMS{

    fmat x;         // State Matrix                         [N x 1]
    fmat F;         // Process Model Jacobian               [N x N]
    fmat H;         // Measurement Model Jacobian           [M x N]

    fmat P;         // Predicted State Error Covariance     [N x N]
    fmat Q;         // Process Noise Covariance             [N x N]
    fmat R;         // Measurement Error Covariance         [M x M]

    fmat K;         // Kalman gain                          [N x M]


} EKF_PARAMS;


typedef struct PID_PARAMS{
     double dt;
     double max;
     double min;
     double Kp;
     double Kd;
     double Ki;
     double pre_error;
     double integral;
}PID_PARAMS;


/** SECTION:

     MESSAGES TYPES

*/

typedef struct RC_COMMAND_MSG{
     int32_t yaw;
     int32_t pitch;
     int32_t speed;
}RC_COMMAND_MSG;


typedef struct CommunicationHeaderByte{
     int32_t header;
     int32_t msg_type;
     int32_t data_type;
     int32_t measurement_type;
     int32_t measurement_length;
}CommunicationHeaderByte;

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

#endif // PARAMS_H_
