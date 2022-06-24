#ifndef DIFFDRIVE_ROBOCLAW_H_
#define DIFFDRIVE_ROBOCLAW_H_

#include <vector>
#include <chrono>
#include <mutex>
#include "devices/roboclaw.h"

using namespace std;
using namespace chrono;

class DiffDriveClaw {
private:
     std::mutex _lock;
     /** Communication Variables */
     int _pi;
     vector<int> _ser_handles;

     /** Roboclaw Config Parameters */
     float _max_speed;
     float _base_width;
     int _qpps_per_meter;
     float _wheel_diameter;

     /** Roboclaw Data Containers */
     float _main_battery_voltage;
     vector<float> _motor_currents;
     vector<uint16_t> _claw_error_status;
     vector<uint32_t> _motors_pps;
     vector<float> _motor_velocities;
     vector<uint32_t> _encoder_positions;

     /** Timing Variables */
     float _sampled_enc_dt;
     high_resolution_clock::time_point _prev_enc_time;

     /** Odometry Update Related Variables */
     uint32_t _prev_encoder_positions[2] = {0,0};
     float _odom_changes[2] = {0.0,0.0};
     float _cur_pose[3] = {0.0, 0.0, 0.0};
     float _linear_vel;
     float _angular_vel;
     /** Variables for motor direction switching */
     int _cmd_turn_dir_sign;
     int _odom_turn_dir_sign;
     int _left_dir;
     int _right_dir;
public:
     /**RoboClaw* leftclaw;
     RoboClaw* rightclaw;**/

	 RoboClaw* claw;

     /** Constructors / Deconstructors */
     DiffDriveClaw();
     DiffDriveClaw(int pi);
     DiffDriveClaw(const char* config_file);
     DiffDriveClaw(int pi, const char* config_file);
     ~DiffDriveClaw();

     /** Device Startup Functions */
     int init(const char* config_file);
     /** int init(const char* serial_device, int baud, int left_claw_addr = 128, int right_claw_addr = 129);*/
	int init(const char* serial_device, int baud, int claw_addr = 128);
     /** int init(int baud, const char* serial_device_left, const char* serial_device_right, int left_claw_addr = 128, int right_claw_addr = 129);*/
	int init(int buad, const char* serial_device, int claw_addr = 128);

     /** Device Motion Control Functions */
     vector<int32_t> get_target_speeds(float v, float w);
     void drive(float v, float w);
     void drive(vector<int32_t> cmds);
     /** Device Data Request/Update Functions */
     void update_status(bool verbose = false);
     void update_motors(bool verbose = false);
     void update_odometry(bool verbose = false);

     /** Class Config Getters / Setters */
     void set_odom_turn_direction(int dir);
     void set_command_turn_direction(int dir);
     void set_base_width(float width);
     void set_max_speed(float speed);
     void set_qpps_per_meter(int qpps);
     void set_wheel_diameter(float diameter);
     float get_base_width();
     float get_max_speed();
     int get_qpps_per_meter();
     float get_wheel_diameter();

     /** Class Stored Data Getters */
     vector<float> get_currents();
     float get_voltage();
     vector<uint16_t> get_error_status();
     vector<uint32_t> get_encoder_positions();
     vector<float> get_motor_speeds();
     vector<uint32_t> get_motor_pps();
     vector<float> get_odom_deltas();
     vector<float> get_pose();
     vector<float> get_velocities();

     /** Helper Functions */
     void reset_encoders(bool verbose = false);
     void reset_odometry(bool verbose = false);
     float normalize_heading(const float& angle);
};

#endif // DIFFDRIVE_ROBOCLAW_H_
