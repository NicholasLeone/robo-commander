#include <thread>
#include <pigpiod_if2.h>

#include "utils/utils.h"
#include "dual_roboclaw.h"

using namespace std;

DualClaw::DualClaw(int pi){

     this->_pi = pi;

     /**************************************************************************
     * LOAD CONFIG FILE
     **************************************************************************/
     std::map<std::string, float> variables;
     LoadInitialVariables("../../config/profiles/dualclaw.config", variables);

     char* ser_path = (string) variables["dev"];//ser_path.c_str()
     int baud = (int) variables["baud"];
     _base_width = variables["base_width"];
     _max_speed = variables["max_speed"];
     _qpps_per_meter = variables["qpps_per_meter"];
     // TODO: Add more tune-able parameters

     // printf("%d %d %d %d %d %d %d %d\r\n",fr_pwm,fr_dir,fl_pwm,fl_dir,rr_pwm,rr_dir,rl_pwm,rl_dir);

     /**************************************************************************
     * END LOAD CONFIG FILE
     **************************************************************************/

     _ser_handle = serial_open(pi, ser_path, baud, 0);

     leftclaw = new RoboClaw(pi, ser_handle, 128);
     rightclaw = new RoboClaw(pi, ser_handle, 129);
}


DualClaw::~DualClaw(){

     vector<int32_t> cmds{0,0};
     drive(cmds);

     delete leftclaw;
     delete rightclaw;

     int err = serial_close(_pi, ser_handle);
     usleep(1 * 10000000);
}

vector<int32_t> DualClaw::update_commands(float v, float w){

     vector<int32_t> cmds(2);

     /**   Differential Drive Drive Equations     */
     float vLeft = v - w * centerToWheelRadius;
     float vRight = v + w * centerToWheelRadius;
     target_speed_left = vLeft;
     target_speed_right = vRight;

     int32_t leftClawSpd = vLeft * max_speed;
     int32_t rightClawSpd = vRight * max_speed;

     // TODO: Make adaptable to variable number of motors (also account for motor order)
     cmds[0] = leftClawSpd;
     cmds[1] = rightClawSpd;

     // printf("V, Omega, Left, Right:     %.5f   |    %.5f |    %.5f |    %.5f \r\n",v,w,vLeft,vRight);

}

void DualClaw::drive(vector<int32_t> cmds){

     int n = cmds.size();

     leftclaw->SpeedM1M2(cmds.at(0),cmds.at(0));
     rightclaw->SpeedM1M2(cmds.at(1),cmds.at(1));

}

void DualClaw::updateSensors(){

     // now = rospy.Time.now()
     //    dt = now.to_sec() - self.last_odom.to_sec()
     //
     //    if dt > 10.0 or dt == 0.0:
     //        self.last_odom = now
     //        return
     //
     //    self.last_odom = now
     //
     //    enc1 = self.roboclaw.ReadEncM1(ADDRESS)
     //    enc2 = self.roboclaw.ReadEncM2(ADDRESS)
     //
     //    encoder_left = 0
     //    encoder_right = 0
     //
     //    if enc1[0] == 1:
     //        encoder_left = self.left_dir * enc1[1]
     //    else:
     //        rospy.logerr("failed to read encoder 1")
     //        return
     //
     //    if enc2[0] == 1:
     //        encoder_right = self.right_dir * enc2[1]
     //    else:
     //        rospy.logerr("failed to read encoder 2")
     //        return
     //
     //    dist_left = 0.0
     //    dist_right = 0.0
     //
     //    if self.robot_dir:
     //        dist_left = float(encoder_left - self.last_enc_left) / self.ticks_per_m
     //        dist_right = float(encoder_right - self.last_enc_right) / self.ticks_per_m
     //    else:
     //        dist_right = float(encoder_left - self.last_enc_left) / self.ticks_per_m
     //        dist_left = float(encoder_right - self.last_enc_right) / self.ticks_per_m
     //
     //    self.last_enc_left = encoder_left
     //    self.last_enc_right = encoder_right
     //
     //    distance_travelled = (dist_left + dist_right) / 2.0
     //    delta_th = (dist_right - dist_left) / self.base_width
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

     // printf("V1, V2, V3, V4:     %.5f   |    %.5f |    %.5f |    %.5f \r\n",spd1,spd2,spd3,spd4);

}
