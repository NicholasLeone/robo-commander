#ifndef ANDROID_APP_INTERFACE_H_
#define ANDROID_APP_INTERFACE_H_

#include <chrono>
#include "communication/udp.h"

using namespace std;

typedef struct Sim_Msg_Data_Header{
     union{
          int32_t component_id;
          int32_t header;
     };
     int32_t msg_type;
     int32_t data_type;
     int32_t measurement_type;
     int32_t measurement_length;
}Sim_Msg_Data_Header;

typedef struct Sim_Msg_MotionCommands{
     int32_t normalized_yaw_rate;
	int32_t padding;
     int32_t normalized_speed;
}Sim_Msg_MotionCommands;

typedef struct Sim_CameraGimbalsCommand{
	int16_t front_angle;
	int16_t left_angle;
	int16_t right_angle;
	int16_t back_angle;
}Sim_CameraGimbalsCommand;

typedef enum SimulatorMessageType{
	SIMULATOR_MESSAGE_SET_PORT = 0,
	SIMULATOR_MESSAGE_CONTROL = 1,
	SIMULATOR_MESSAGE_DATA_GPS = 2,
	SIMULATOR_MESSAGE_DATA_GYRO = 3,
	SIMULATOR_MESSAGE_DATA_ACCELERATION = 4,
	SIMULATOR_MESSAGE_DATA_RCCOMMANDS = 5,
	SIMULATOR_MESSAGE_DATA_LIDAR = 6,
	SIMULATOR_MESSAGE_DATA_ORIENTATION = 7,
	SIMULATOR_MESSAGE_DATA_ENCODER = 8,
	SIMULATOR_MESSAGE_DATA_FRONT_CAM = 9,
	SIMULATOR_MESSAGE_DATA_IMU = 9,
	SIMULATOR_MESSAGE_DATA_BATTERY = 10,
	SIMULATOR_MESSAGE_DATA_CAMERA_GIMBALS = 11,
	SIMULATOR_MESSAGE_DATA_MOTION_CONTROL = 12,
     SIMULATOR_MESSAGE_DATA_COLLECTION = 13,
     SIMULATOR_MESSAGE_DATA_START_STOP_AUTO_MODE = 14,
     SIMULATOR_MESSAGE_DATA_START_STOP_IGNORE_OBSTACLE = 15,
}SimulatorMessageType;

typedef struct AndroidInterfaceData{
	int16_t front_cam_angle;
	int16_t left_cam_angle;
	int16_t right_cam_angle;
	int16_t back_cam_angle;
     float normalized_speed;
     float normalized_turn_rate;
}AndroidInterfaceData;

class AndroidAppInterface {
private:
     int _port;

     int _count;
     int _header_count;
     int _rc_msg_count;
	int _gimbal_msg_count;

     chrono::high_resolution_clock::time_point _prev_time;
     chrono::high_resolution_clock::time_point _cur_time;

     bool flag_verbose;
     bool debug_timing;

public:
     UDP* mUdp;

     AndroidInterfaceData mData;
     Sim_Msg_Data_Header header;
     Sim_Msg_MotionCommands controls;
	Sim_CameraGimbalsCommand angles;

     /** Class Initialization */
     AndroidAppInterface();
     AndroidAppInterface(int listenPort, bool verbose = false);
     ~AndroidAppInterface();

     /** Updated */
     void readHeader(char* msgBuffer);
     void readRC(char* msgBuffer);
     void readGimbalAngles(char* msgBuffer);
     int receiveUdpMessage(bool verbose = false, bool debug = false);

     /** Getters and Setters */
     AndroidInterfaceData getReceivedData();

     /** Debugging */
     void print_udp_header(Sim_Msg_Data_Header header);

     /** Deprecated */
     void read_udp_header();
     void read_udp_commands();
};

#endif // ANDROID_APP_INTERFACE_H_
