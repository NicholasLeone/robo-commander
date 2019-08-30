#ifndef SWANSONV2_H_
#define SWANSONV2_H_

#include <fstream>
#include <chrono>

#include "communication/udp.h"
#include "vehicle_profiles/dual_roboclaw.h"
#include "sensors/imu.h"

using namespace std;
using namespace chrono;

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

typedef struct Sim_Msg_CameraGimbalsCommand{
	float front_angle;
	float left_angle;
	float right_angle;
}Sim_Msg_CameraGimbalsCommand;

// typedef struct Sim_CameraGimbalsCommand{
// 	float front_angle;
// 	float left_angle;
// 	float right_angle;
// }Sim_CameraGimbalsCommand;

typedef struct Sim_CameraGimbalsCommand{
	int16_t front_angle;
	int16_t left_angle;
	int16_t right_angle;
	int16_t back_angle;
}Sim_CameraGimbalsCommand;

typedef struct Sim_Msg_MotorSpeedCommand{
	float angular_speed;
}Sim_Msg_MotorSpeedCommand;

typedef struct Sim_Msg_MotorPositionCommand{
	float angular_position;
}Sim_Msg_MotorPositionCommand;

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


class SwansonV2 {

private:

     int _pi;
     int _port;
     int NStates = 20;

     float max_speed;
     float centerToWheelRadius;

     system_clock::time_point current_system_time;
     system_clock::time_point previous_system_time;
     system_clock::time_point start_system_time;
     float _time;

     ofstream datalog;

     int _count;
     int _header_count;
     int _rc_msg_count;
	int _gimbal_msg_count;
     float _max_speed;
     float _max_omega;

     bool flag_verbose;
public:

     UDP* rc_in;
     RC_COMMAND_MSG _controls;
     Udp_Msg_Header udp_header;

     Sim_Msg_Data_Header _header;
     Sim_Msg_MotionCommands controls;
	Sim_CameraGimbalsCommand angles;

	void readHeader(char* msgBuffer);
     void readRC(char* msgBuffer);
     void readGimbalAngles(char* msgBuffer);
     void receiveUdpMessage();


     DualClaw* claws;
     IMU* imu;

     // FUNCTIONS
	SwansonV2(int pi);
     ~SwansonV2();

     void read_udp_header();
     void read_udp_commands();

     void drive(float v, float w);
     void update_sensors();
     vector<float> get_sensor_data();

     void open_datalog(string file_path);
     void close_datalog();
     void add_datalog_entry(vector<float> data);

     void print_udp_header(Sim_Msg_Data_Header header);
};


#endif // SWANSONV2_H_
