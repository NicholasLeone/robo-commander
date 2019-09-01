#ifndef DUAL_ROBOCLAW_H_
#define DUAL_ROBOCLAW_H_

#include <vector>
#include "devices/roboclaw.h"

using namespace std;

class DualClaw {

private:

     int _pi;
     int _ser_handle;
     int _address[2];
     uint8_t _status[4];

     float _max_speed;
     float _base_width;
     float _wheel_diameter;
     int _qpps_per_meter;

     bool _valid[4];

     uint16_t _main_battery[2] = {0,0};
     int16_t _currents[4] = {0,0,0,0};
     uint32_t _positions[4] = {0,0,0,0};
     uint32_t _speeds[4] = {0,0,0,0};

     uint32_t _last_positions[4] = {0,0,0,0};

     float _current_pose[3] = {0,0,0};

public:
     RoboClaw* leftclaw;
     RoboClaw* rightclaw;

     int flag_turn_dir;
     int flag_left_sign;
     int flag_right_sign;

     uint32_t qpps[4];
     float kp[4];
     float ki[4];
     float kd[4];

     uint16_t error[2];
     float main_battery[2];
     float currents[4];

     float speeds[4];
     float positions[4];
     float dx;
     float dy;
     float dyaw;
     float dist_traveled;

     DualClaw();
     DualClaw(int pi);
     // DualClaw(int pi, int serial_handle);
     DualClaw(int pi, const char* config_file);
     DualClaw(const char* config_file);
     ~DualClaw();

     // FUNCTIONS
     int init(const char* config_file);
     int init(const char* serial_device, int baud, int left_claw_addr = 128, int right_claw_addr = 129);

     void drive(float v, float w);
     void drive(vector<int32_t> cmds);
     void update_status();
     void update_encoders();

     vector<int32_t> get_target_speeds(float v, float w);

     void set_turn_direction(int dir);
     void set_base_width(float width);
     void set_max_speed(float speed);
     void set_qpps_per_meter(int qpps);
     void set_wheel_diameter(float diameter);

     float get_base_width();
     float get_max_speed();
     int get_qpps_per_meter();
     float get_wheel_diameter();

     vector<float> get_currents();
     vector<float> get_voltages();
     vector<uint32_t> get_encoder_positions();
     vector<float> get_encoder_speeds();
     vector<float> get_odom_deltas();
     vector<float> get_pose();

     void reset_encoders();

     // TODO: develop
     // void keep_alive();
};


// // GOES INTO ROS NODE
//
// now = rospy.Time.now()
//    dt = now.to_sec() - self.last_odom.to_sec()
//
//    if dt > 10.0 or dt == 0.0:
//        self.last_odom = now
//        return
//
//    self.last_odom = now
//
// dualclaw.get_odom();
//
//    self.vx = distance_travelled / dt
//    self.vth = delta_th / dt
//
//    if distance_travelled != 0.0:
//        delta_x = cos(delta_th) * distance_travelled
//        delta_y = -sin(delta_th) * distance_travelled
//        self.x += (cos(self.theta) * delta_x - sin(self.theta) * delta_y)
//        self.y += (sin(self.theta) * delta_x - cos(self.theta) * delta_y)
//
//    if delta_th != 0.0:
//        self.theta += delta_th


#endif // DUAL_ROBOCLAW_H_
