#include <pigpiod_if2.h>
#include <math.h>

#include "utilities/utils.h"
#include "drivetrains/dual_roboclaw.h"

using namespace std;

DualClaw::DualClaw(){
     int pi = pigpio_start(NULL, NULL);
     if(pi < 0){
          printf("[ERROR] DualClaw() --- Could not initialize pigpiod \r\n");
		exit(0);
     }
     this->_pi = pi;

     flag_turn_dir = 1;
     flag_left_sign = 1;
     flag_right_sign = 1;

     _base_width = 0.219;
     _max_speed = 2.0;

     _qpps_per_meter = 9596;
     _wheel_diameter = 0.1905;
}

DualClaw::DualClaw(int pi){
     this->_pi = pi;

     flag_turn_dir = 1;
     flag_left_sign = 1;
     flag_right_sign = 1;

     _base_width = 0.219;
     _max_speed = 2.0;

     _qpps_per_meter = 9596;
     _wheel_diameter = 0.1905;
}

// DualClaw::DualClaw(int pi, int serial_handle){
//      this->_pi = pi;
//      this->_ser_handle = serial_handle;
//      flag_turn_dir = 1;
//      flag_left_sign = 1;
//      flag_right_sign = 1;
//
//      _base_width = 0.219;
//      _max_speed = 2.0;
//
//      _qpps_per_meter = 9596;
//      _wheel_diameter = 0.1905;
// }

DualClaw::DualClaw(int pi, const char* config_file){
     this->_pi = pi;

     flag_turn_dir = 1;
     flag_left_sign = 1;
     flag_right_sign = 1;

     int err = this->init(config_file);
     if(err < 0){
          printf("[ERROR] DualClaw() --- Could not initialize communication with one or more RoboClaws. Error code = %d\r\n",err);
          exit(0);
     }
}

DualClaw::DualClaw(const char* config_file){
     int pi = pigpio_start(NULL, NULL);
     if(pi < 0){
          printf("[ERROR] DualClaw() --- Could not initialize pigpiod \r\n");
		exit(0);
     }
     this->_pi = pi;

     flag_turn_dir = 1;
     flag_left_sign = 1;
     flag_right_sign = 1;

     int err = this->init(config_file);
     if(err < 0){
          printf("[ERROR] DualClaw() --- Could not initialize communication with one or more RoboClaws. Error code = %d\r\n",err);
		exit(0);
     }
}

DualClaw::~DualClaw(){
     vector<int32_t> cmds{0,0};
     drive(cmds);

     delete leftclaw;
     delete rightclaw;

     int err = serial_close(_pi, _ser_handle);
     usleep(1 * 10000000);
}

int DualClaw::init(const char* config_file){
     /**************************************************************************
     * LOAD CONFIG FILE
     **************************************************************************/
     std::map<std::string, std::string> variables;
     LoadStringVariables(config_file, variables);

     string _ser_path = variables["dev"];
     char* ser_path = (char*) _ser_path.c_str();
     int baud = stoi(variables["baud"]);
     _base_width = stof(variables["base_width"]);
     _max_speed = stof(variables["max_speed"]);
     _qpps_per_meter = stoi(variables["qpps_per_meter"]);
     _wheel_diameter = stof(variables["wheel_diameter"]);

     // TODO: Add more tune-able parameters

     // printf("%d %d %d %d %d %d %d %d\r\n",fr_pwm,fr_dir,fl_pwm,fl_dir,rr_pwm,rr_dir,rl_pwm,rl_dir);
     printf("ROBOCLAW SETTINGS: \r\n");
     printf("       Device Address: %s\r\n", ser_path);
     printf("       Claw Baud Rate: %d\r\n", baud);
     printf("       Base Width: %.4f\r\n", _base_width);
     printf("       Max Speed (m/s): %.3f\r\n", _max_speed);
     printf("       QPPS per Meter: %d\r\n", _qpps_per_meter);
     printf("       Wheel Diameter (m): %f\r\n", _wheel_diameter);
     printf("\r\n");

     /**************************************************************************
     * END LOAD CONFIG FILE
     **************************************************************************/

     int h = serial_open(this->_pi, ser_path, baud, 0);
     if(h < 0){
          printf("[ERROR] Could not initialize pigpiod serial device %s using baud of %d\r\n",ser_path,baud);
          return -1;
     }
     this->_ser_handle = h;

     leftclaw = new RoboClaw(this->_pi, h, 128);
     rightclaw = new RoboClaw(this->_pi, h, 129);

     leftclaw->ReadM1VelocityPID(kp[0],ki[0],kd[0],qpps[0]);
     leftclaw->ReadM2VelocityPID(kp[1],ki[1],kd[1],qpps[1]);
     rightclaw->ReadM1VelocityPID(kp[2],ki[2],kd[2],qpps[2]);
     rightclaw->ReadM2VelocityPID(kp[3],ki[3],kd[3],qpps[3]);

     printf("Current Pose [X (m), Y (m), Yaw (rad)]: %.3f     |    %.3f   |       %.3f\r\n",_current_pose[0],_current_pose[1],_current_pose[5]);

     printf("[MOTOR 1]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[0],ki[0],kd[0],qpps[0]);
     printf("[MOTOR 2]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[1],ki[1],kd[1],qpps[1]);
     printf("[MOTOR 3]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[2],ki[2],kd[2],qpps[2]);
     printf("[MOTOR 4]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[3],ki[3],kd[3],qpps[3]);
     printf("\r\n");
     this->reset_encoders();
     return 0;
}

int DualClaw::init(const char* serial_device, int baud,int left_claw_addr,int right_claw_addr){
     int h = serial_open(this->_pi, const_cast <char*>(serial_device), baud, 0);
     if(h < 0){
          printf("[ERROR] Could not initialize pigpiod serial device %s using baud of %d\r\n",serial_device,baud);
          return -1;
     }
     this->_ser_handle = h;

     leftclaw = new RoboClaw(this->_pi, h, left_claw_addr);
     rightclaw = new RoboClaw(this->_pi, h, right_claw_addr);

     leftclaw->ReadM1VelocityPID(kp[0],ki[0],kd[0],qpps[0]);
     leftclaw->ReadM2VelocityPID(kp[1],ki[1],kd[1],qpps[1]);
     rightclaw->ReadM1VelocityPID(kp[2],ki[2],kd[2],qpps[2]);
     rightclaw->ReadM2VelocityPID(kp[3],ki[3],kd[3],qpps[3]);

     printf("Current Pose [X (m), Y (m), Yaw (rad)]: %.3f     |    %.3f   |       %.3f\r\n",_current_pose[0],_current_pose[1],_current_pose[5]);

     printf("[MOTOR 1]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[0],ki[0],kd[0],qpps[0]);
     printf("[MOTOR 2]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[1],ki[1],kd[1],qpps[1]);
     printf("[MOTOR 3]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[2],ki[2],kd[2],qpps[2]);
     printf("[MOTOR 4]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[3],ki[3],kd[3],qpps[3]);
     printf("\r\n");
     this->reset_encoders();
     return 0;
}

void DualClaw::drive(vector<int32_t> cmds){
     int n = cmds.size();

     leftclaw->SpeedM1M2(cmds.at(0),cmds.at(0));
     rightclaw->SpeedM1M2(cmds.at(1),cmds.at(1));
}

void DualClaw::drive(float v, float w){
     vector<int32_t> cmds = this->get_target_speeds(v,w);
     int n = cmds.size();

     leftclaw->SpeedM1M2(cmds.at(0),cmds.at(0));
     rightclaw->SpeedM1M2(cmds.at(1),cmds.at(1));
}

vector<int32_t> DualClaw::get_target_speeds(float v, float w){
     vector<int32_t> cmds(2);

     /**   Differential Drive Drive Equations     */
     float v_left = v - (flag_turn_dir * w) * (_base_width / 2.0);
     float v_right = v + (flag_turn_dir * w) * (_base_width / 2.0);

     float left_spd = v_left * _qpps_per_meter * flag_left_sign;
     float right_spd = v_right * _qpps_per_meter * flag_right_sign;

     // printf("Left Velocity, Right Velocity:     %.5f   |    %.5f \r\n",left_spd, right_spd);

     // TODO: Make adaptable to variable number of motors (also account for motor order)
     cmds[0] = (int32_t)(left_spd);
     cmds[1] = (int32_t)(right_spd);

     return cmds;
}

void DualClaw::set_turn_direction(int dir){ this->flag_turn_dir = dir; }

void DualClaw::set_base_width(float width){ this->_base_width = width; }
void DualClaw::set_max_speed(float speed){ this->_max_speed = speed; }
void DualClaw::set_qpps_per_meter(int qpps){ this->_qpps_per_meter = qpps; }
void DualClaw::set_wheel_diameter(float diameter){ this->_wheel_diameter = diameter; }

float DualClaw::get_base_width(){ return this->_base_width; }
float DualClaw::get_max_speed(){ return this->_max_speed; }
int DualClaw::get_qpps_per_meter(){ return this->_qpps_per_meter; }
float DualClaw::get_wheel_diameter(){ return this->_wheel_diameter; }

vector<float> DualClaw::get_currents(){
     vector<float> tmp {currents[0],currents[1],currents[2],currents[3]};
     return tmp;
}

vector<float> DualClaw::get_voltages(){
     vector<float> tmp {main_battery[0],main_battery[1]};
     return tmp;
}

vector<uint32_t> DualClaw::get_encoder_positions(){
     vector<uint32_t> tmp {_positions[0],_positions[1],_positions[2],_positions[3]};
     return tmp;
}

vector<float> DualClaw::get_encoder_speeds(){
     vector<float> tmp {speeds[0],speeds[1],speeds[2],speeds[3]};
     return tmp;
}

vector<float> DualClaw::get_odom_deltas(){
     vector<float> tmp {dist_traveled,dyaw,dx,dy};
     return tmp;
}

vector<float> DualClaw::get_pose(){
     vector<float> tmp {_current_pose[0],_current_pose[1],_current_pose[2]};
     return tmp;
}

void DualClaw::update_status(){

     int err[8];
     uint8_t status;
     bool valid1,valid2,valid3,valid4;

     _main_battery[0] = leftclaw->ReadMainBatteryVoltage(&valid1);
     _main_battery[1] = rightclaw->ReadMainBatteryVoltage(&valid2);

     main_battery[0] = (float) ((int16_t) _main_battery[0]) / 10.0;
     main_battery[1] = (float) ((int16_t) _main_battery[1]) / 10.0;

     err[1] = leftclaw->ReadCurrents(_currents[0], _currents[1]);
     err[1] = rightclaw->ReadCurrents(_currents[2], _currents[3]);

     currents[0] = (float) _currents[0] / 100.0;
     currents[1] = (float) _currents[1] / 100.0;
     currents[2] = (float) _currents[2] / 100.0;
     currents[3] = (float) _currents[3] / 100.0;

     // printf("Battery Voltages:     %.3f |    %.3f\r\n",main_battery[0], main_battery[1]);
     // printf("Motor Currents:     %.3f |    %.3f |    %.3f |    %.3f\r\n",currents[0], currents[1], currents[2], currents[3]);

     // TODO: User-friendly handling of errors
     error[0] = leftclaw->ReadError(&valid3);
     error[1] = rightclaw->ReadError(&valid4);

}

void DualClaw::update_encoders(){

     uint8_t status1, status2, status3, status4;
     bool valid1, valid2, valid3, valid4;

     uint32_t tmpPos[4] = {0,0,0,0};
     float tmpDist[4] = {0,0,0,0};

     _speeds[0] = leftclaw->ReadSpeedM1(&status1,&valid1);
     _speeds[1] = leftclaw->ReadSpeedM2(&status2,&valid2);
     _speeds[2] = rightclaw->ReadSpeedM1(&status3,&valid3);
     _speeds[3] = rightclaw->ReadSpeedM2(&status4,&valid4);

     for(int i = 0; i <= 3;i++){
          speeds[i] = (float) ((int32_t) _speeds[i]) / _qpps_per_meter;
     }

     leftclaw->ReadEncoders(_positions[0],_positions[1]);
     rightclaw->ReadEncoders(_positions[2],_positions[3]);

     tmpPos[0] = _positions[0] * flag_left_sign;
     tmpPos[1] = _positions[1] * flag_left_sign;
     tmpPos[2] = _positions[2] * flag_right_sign;
     tmpPos[3] = _positions[3] * flag_right_sign;

     for(int i = 0; i <= 3;i++){
          int32_t dPos = (int32_t) tmpPos[i] - (int32_t) _last_positions[i];
          tmpDist[i] = (float) dPos / _qpps_per_meter;
          _last_positions[i] = tmpPos[i];
     }

     float avg_dist_l = (tmpDist[0] + tmpDist[1]) / 2.0;
     float avg_dist_r = (tmpDist[2] + tmpDist[3]) / 2.0;

     dist_traveled = (avg_dist_l + avg_dist_r) / 2.0;
     dyaw = (avg_dist_r - avg_dist_l) / _base_width;

     // Update current yaw first for back-solving changes in the X and Y points
     _current_pose[2] = _current_pose[2] + dyaw;

     // Compute changes in X and Y coordinates
     dx = dist_traveled * cos(_current_pose[2]);
     dy = dist_traveled * sin(_current_pose[2]);

     // Update current 2D location
     _current_pose[0] = _current_pose[0] + dx;
     _current_pose[1] = _current_pose[1] + dy;

     // printf("Motor Speeds (m/s)/[PPS]:  %.3f / (%d)  | %.3f / (%d)  | %.3f / (%d)  | %.3f / (%d)\r\n",speeds[0],_speeds[0],speeds[1],_speeds[1],speeds[2],_speeds[2],speeds[3],_speeds[3]);
     // printf("Encoder Positions (qpps): %d | %d | %d | %d\r\n",tmpPos[0],tmpPos[1],tmpPos[2],tmpPos[3]);
     // printf("Δdistance, ΔX, ΔY, ΔYaw: %.3f, %.3f, %.3f, %.3f\r\n",dist_traveled, dx,dy,dyaw);
     // printf("Current Pose [X (m), Y (m), Yaw (rad)]: %.3f     |    %.3f   |       %.3f\r\n",_current_pose[0],_current_pose[1],_current_pose[5]);
}

void DualClaw::reset_encoders(){
     for(int i = 0; i <= 3;i++){ _last_positions[i] = 0; }

     leftclaw->ResetEncoders();
     rightclaw->ResetEncoders();

     leftclaw->ReadEncoders(_positions[0],_positions[1]);
     rightclaw->ReadEncoders(_positions[2],_positions[3]);

     printf("Encoders Reset (qpps): %d | %d | %d | %d\r\n",_positions[0],_positions[1],_positions[2],_positions[3]);
}
