#ifndef SWANSONV2_H_
#define SWANSONV2_H_

#include <fstream>
#include <chrono>

#include "communication/udp.h"
#include "interfaces/android_app_interface.h"
#include "drivetrains/dual_roboclaw.h"
#include "sensors/generic_rtimu.h"

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

     int _count;
     int _header_count;
     int _rc_msg_count;
	int _gimbal_msg_count;
	int timeoutCnt;
     float _max_speed;
     float _max_omega;

     bool flag_verbose;
public:

     AndroidInterfaceData cmdData;
     AndroidAppInterface* mRelay;

     DualClaw* claws;
     GenericRTIMU* imu;

     // FUNCTIONS
	SwansonV2(int pi);
     ~SwansonV2();

     void drive(float v, float w);
     void update_sensors();
     void update_control_interface();
     vector<float> get_sensor_data();

     void open_datalog(string file_path);
     void close_datalog();
     void add_datalog_entry(vector<float> data);
};


#endif // SWANSONV2_H_
