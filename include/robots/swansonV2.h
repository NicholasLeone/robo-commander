#ifndef SWANSONV2_H_
#define SWANSONV2_H_

#include <fstream>
#include <chrono>

#include "comms/udp.h"
#include "profiles/dual_roboclaw.h"
#include "sensors/imu.h"

using namespace std;
using namespace chrono;

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

public:

     UDP* rc_in;
     RC_COMMAND_MSG controls;
     Udp_Msg_Header udp_header;

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

     void print_udp_header(Udp_Msg_Header* header);
};


#endif // SWANSONV2_H_
