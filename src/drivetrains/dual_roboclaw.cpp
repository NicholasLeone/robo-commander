#include <pigpiod_if2.h>
#include <math.h>

#include "utilities/utils.h"
#include "drivetrains/dual_roboclaw.h"

using namespace std;
using namespace chrono;

DualClaw::DualClaw(){
     int pi = pigpio_start(NULL, NULL);
     if(pi < 0){
          printf("[ERROR] DualClaw() --- Could not initialize pigpiod \r\n");
		exit(0);
     }
     this->_pi = pi;

     this->flag_turn_dir = 1;
     this->flag_left_sign = 1;
     this->flag_right_sign = 1;

     this->_base_width = 0.219;
     this->_max_speed = 2.0;

     this->_qpps_per_meter = 9596;
     this->_wheel_diameter = 0.1905;
     this->_prev_time = high_resolution_clock::now();
}

DualClaw::DualClaw(int pi){
     this->_pi = pi;

     this->flag_turn_dir = 1;
     this->flag_left_sign = 1;
     this->flag_right_sign = 1;

     this->_base_width = 0.219;
     this->_max_speed = 2.0;

     this->_qpps_per_meter = 9596;
     this->_wheel_diameter = 0.1905;
     this->_prev_time = high_resolution_clock::now();
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

     this->flag_turn_dir = 1;
     this->flag_left_sign = 1;
     this->flag_right_sign = 1;

     int err = this->init(config_file);
     if(err < 0){
          printf("[ERROR] DualClaw() --- Could not initialize communication with one or more RoboClaws. Error code = %d\r\n",err);
          exit(0);
     }
     this->_prev_time = high_resolution_clock::now();
}

DualClaw::DualClaw(const char* config_file){
     int pi = pigpio_start(NULL, NULL);
     if(pi < 0){
          printf("[ERROR] DualClaw() --- Could not initialize pigpiod \r\n");
		exit(0);
     }
     this->_pi = pi;

     this->flag_turn_dir = 1;
     this->flag_left_sign = 1;
     this->flag_right_sign = 1;

     int err = this->init(config_file);
     if(err < 0){
          printf("[ERROR] DualClaw() --- Could not initialize communication with one or more RoboClaws. Error code = %d\r\n",err);
		exit(0);
     }
     this->_prev_time = high_resolution_clock::now();
}

DualClaw::~DualClaw(){
     printf("[INFO] DualClaw() --- Shutting Down...\r\n");
     vector<int32_t> cmds{0,0};
     this->drive(cmds);

     delete this->leftclaw;
     delete this->rightclaw;

     int err = serial_close(this->_pi, this->_ser_handle);
     // usleep(1 * 10000000);
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
     this->_base_width = stof(variables["base_width"]);
     this->_max_speed = stof(variables["max_speed"]);
     this->_qpps_per_meter = stoi(variables["qpps_per_meter"]);
     this->_wheel_diameter = stof(variables["wheel_diameter"]);

     // TODO: Add more tune-able parameters

     // printf("%d %d %d %d %d %d %d %d\r\n",fr_pwm,fr_dir,fl_pwm,fl_dir,rr_pwm,rr_dir,rl_pwm,rl_dir);
     printf("ROBOCLAW SETTINGS: \r\n");
     printf("       Device Address: %s\r\n", ser_path);
     printf("       Claw Baud Rate: %d\r\n", baud);
     printf("       Base Width: %.4f\r\n", this->_base_width);
     printf("       Max Speed (m/s): %.3f\r\n", this->_max_speed);
     printf("       QPPS per Meter: %d\r\n", this->_qpps_per_meter);
     printf("       Wheel Diameter (m): %f\r\n", this->_wheel_diameter);
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

     this->leftclaw = new RoboClaw(this->_pi, h, 128);
     this->rightclaw = new RoboClaw(this->_pi, h, 129);

     this->leftclaw->ReadM1VelocityPID(this->kp[0],this->ki[0],this->kd[0],this->qpps[0]);
     this->leftclaw->ReadM2VelocityPID(this->kp[1],this->ki[1],this->kd[1],this->qpps[1]);
     this->rightclaw->ReadM1VelocityPID(this->kp[2],this->ki[2],this->kd[2],this->qpps[2]);
     this->rightclaw->ReadM2VelocityPID(this->kp[3],this->ki[3],this->kd[3],this->qpps[3]);
     // this->leftclaw->ReadM1VelocityPID(&kp[0],&ki[0],&kd[0],&qpps[0]);
     // this->leftclaw->ReadM2VelocityPID(&kp[1],&ki[1],&kd[1],&qpps[1]);
     // this->rightclaw->ReadM1VelocityPID(&kp[2],&ki[2],&kd[2],&qpps[2]);
     // this->rightclaw->ReadM2VelocityPID(&kp[3],&ki[3],&kd[3],&qpps[3]);

     printf("Current Pose [X (m), Y (m), Yaw (rad)]: %.3f     |    %.3f   |       %.3f\r\n",this->_current_pose[0],this->_current_pose[1],this->_current_pose[5]);
     printf("[MOTOR 1]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",this->kp[0],this->ki[0],this->kd[0],this->qpps[0]);
     printf("[MOTOR 2]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",this->kp[1],this->ki[1],this->kd[1],this->qpps[1]);
     printf("[MOTOR 3]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",this->kp[2],this->ki[2],this->kd[2],this->qpps[2]);
     printf("[MOTOR 4]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",this->kp[3],this->ki[3],this->kd[3],this->qpps[3]);
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

     this->leftclaw = new RoboClaw(this->_pi, h, left_claw_addr);
     this->rightclaw = new RoboClaw(this->_pi, h, right_claw_addr);

     this->leftclaw->ReadM1VelocityPID(this->kp[0],this->ki[0],this->kd[0],this->qpps[0]);
     this->leftclaw->ReadM2VelocityPID(this->kp[1],this->ki[1],this->kd[1],this->qpps[1]);
     this->rightclaw->ReadM1VelocityPID(this->kp[2],this->ki[2],this->kd[2],this->qpps[2]);
     this->rightclaw->ReadM2VelocityPID(this->kp[3],this->ki[3],this->kd[3],this->qpps[3]);
     // this->leftclaw->ReadM1VelocityPID(&kp[0],&ki[0],&kd[0],&qpps[0]);
     // this->leftclaw->ReadM2VelocityPID(&kp[1],&ki[1],&kd[1],&qpps[1]);
     // this->rightclaw->ReadM1VelocityPID(&kp[2],&ki[2],&kd[2],&qpps[2]);
     // this->rightclaw->ReadM2VelocityPID(&kp[3],&ki[3],&kd[3],&qpps[3]);

     printf("Current Pose [X (m), Y (m), Yaw (rad)]: %.3f     |    %.3f   |       %.3f\r\n",this->_current_pose[0],this->_current_pose[1],this->_current_pose[5]);
     printf("[MOTOR 1]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",this->kp[0],this->ki[0],this->kd[0],this->qpps[0]);
     printf("[MOTOR 2]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",this->kp[1],this->ki[1],this->kd[1],this->qpps[1]);
     printf("[MOTOR 3]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",this->kp[2],this->ki[2],this->kd[2],this->qpps[2]);
     printf("[MOTOR 4]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",this->kp[3],this->ki[3],this->kd[3],this->qpps[3]);
     printf("\r\n");
     this->reset_encoders();
     return 0;
}

void DualClaw::drive(vector<int32_t> cmds){
     int n = cmds.size();

     this->leftclaw->SpeedM1M2(cmds.at(0),cmds.at(0));
     this->rightclaw->SpeedM1M2(cmds.at(1),cmds.at(1));
}

void DualClaw::drive(float v, float w){
     vector<int32_t> cmds = this->get_target_speeds(v,w);
     int n = cmds.size();

     this->leftclaw->SpeedM1M2(cmds.at(0),cmds.at(0));
     this->rightclaw->SpeedM1M2(cmds.at(1),cmds.at(1));
}

vector<int32_t> DualClaw::get_target_speeds(float v, float w){
     vector<int32_t> cmds(2);

     /**   Differential Drive Drive Equations     */
     float v_left = v - (this->flag_turn_dir * w) * (this->_base_width / 2.0);
     float v_right = v + (this->flag_turn_dir * w) * (this->_base_width / 2.0);

     float left_spd = v_left * this->_qpps_per_meter * this->flag_left_sign;
     float right_spd = v_right * this->_qpps_per_meter * this->flag_right_sign;

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
     vector<float> tmp {this->currents[0],this->currents[1],this->currents[2],this->currents[3]};
     return tmp;
}
vector<float> DualClaw::get_voltages(){
     vector<float> tmp {this->main_battery[0],this->main_battery[1]};
     return tmp;
}
vector<uint32_t> DualClaw::get_encoder_positions(){
     vector<uint32_t> tmp {this->_positions[0],this->_positions[1],this->_positions[2],this->_positions[3]};
     return tmp;
}
vector<float> DualClaw::get_encoder_speeds(){
     vector<float> tmp {this->speeds[0],this->speeds[1],this->speeds[2],this->speeds[3]};
     return tmp;
}
vector<float> DualClaw::get_odom_deltas(){
     vector<float> tmp {this->dist_traveled,this->dyaw,this->dx,this->dy};
     return tmp;
}
vector<float> DualClaw::get_pose(){
     vector<float> tmp {this->_current_pose[0],this->_current_pose[1],this->_current_pose[2]};
     return tmp;
}
vector<float> DualClaw::get_velocities(){
     vector<float> tmp {this->_linear_vel,this->_angular_vel};
     return tmp;
}

void DualClaw::update_status(){
     int err[8];
     uint8_t status;
     bool valid1,valid2,valid3,valid4;

     this->_main_battery[0] = this->leftclaw->ReadMainBatteryVoltage(&valid1);
     this->_main_battery[1] = this->rightclaw->ReadMainBatteryVoltage(&valid2);

     this->main_battery[0] = (float) ((int16_t) this->_main_battery[0]) / 10.0;
     this->main_battery[1] = (float) ((int16_t) this->_main_battery[1]) / 10.0;

     err[1] = this->leftclaw->ReadCurrents(this->_currents[0], this->_currents[1]);
     err[1] = this->rightclaw->ReadCurrents(this->_currents[2], this->_currents[3]);

     this->currents[0] = (float) this->_currents[0] / 100.0;
     this->currents[1] = (float) this->_currents[1] / 100.0;
     this->currents[2] = (float) this->_currents[2] / 100.0;
     this->currents[3] = (float) this->_currents[3] / 100.0;

     // printf("Battery Voltages:     %.3f |    %.3f\r\n",main_battery[0], main_battery[1]);
     // printf("Motor Currents:     %.3f |    %.3f |    %.3f |    %.3f\r\n",currents[0], currents[1], currents[2], currents[3]);

     // TODO: User-friendly handling of errors
     this->error[0] = this->leftclaw->ReadError(&valid3);
     this->error[1] = this->rightclaw->ReadError(&valid4);
}

void DualClaw::update_encoders(){
     uint8_t status1, status2, status3, status4;
     bool valid1, valid2, valid3, valid4;

     float tmpDist[4] = {0,0,0,0};
     uint32_t tmpPos[4] = {0,0,0,0};
     float curX = this->_current_pose[0];
     float curY = this->_current_pose[1];
     float curYaw = this->_current_pose[2];
     if(isnan(curX)) curX = 0.0;
     if(isnan(curY)) curY = 0.0;
     if(isnan(curYaw)) curYaw = 0.0;

     this->_speeds[0] = this->leftclaw->ReadSpeedM1(&status1,&valid1);
     this->_speeds[1] = this->leftclaw->ReadSpeedM2(&status2,&valid2);
     this->_speeds[2] = this->rightclaw->ReadSpeedM1(&status3,&valid3);
     this->_speeds[3] = this->rightclaw->ReadSpeedM2(&status4,&valid4);

     for(int i = 0; i <= 3;i++){
          this->speeds[i] = (float) ((int32_t) this->_speeds[i]) / (float) this->_qpps_per_meter;
     }

     this->leftclaw->ReadEncoders(this->_positions[0],this->_positions[1]);
     this->rightclaw->ReadEncoders(this->_positions[2],this->_positions[3]);
     // Get time of encoder readings
     high_resolution_clock::time_point now = high_resolution_clock::now();
     duration<float> time_span = duration_cast<duration<float>>(now - this->_prev_time);
     float dt = time_span.count();
     // Update previous time for next reading
     this->_prev_time = now;

     tmpPos[0] = this->_positions[0] * this->flag_left_sign;
     tmpPos[1] = this->_positions[1] * this->flag_left_sign;
     tmpPos[2] = this->_positions[2] * this->flag_right_sign;
     tmpPos[3] = this->_positions[3] * this->flag_right_sign;

     for(int i = 0; i <= 3;i++){
          int32_t dPos = (int32_t) tmpPos[i] - (int32_t) this->_last_positions[i];
          tmpDist[i] = ((float) dPos) / (float) this->_qpps_per_meter;
          this->_last_positions[i] = tmpPos[i];
     }

     float avg_dist_l = (tmpDist[0] + tmpDist[1]) / 2.0;
     float avg_dist_r = (tmpDist[2] + tmpDist[3]) / 2.0;

     float linDist = (avg_dist_l + avg_dist_r) / 2.0;
     float dTheta = (avg_dist_r - avg_dist_l) / this->_base_width;
     if(isnan(linDist)) linDist = 0.0;

     // Compute changes in X and Y coordinates
     float _dx, _dy, _yaw;
     if(isnan(dTheta)){
          dTheta = 0.0;
          _dx = linDist * cos(curYaw);
          _dy = linDist * sin(curYaw);
          _yaw = curYaw;
     } else{
          float r = linDist / dTheta;
          _dx = r * (sin(curYaw + dTheta) - sin(curYaw));
          _dy = -r * (cos(curYaw + dTheta) - cos(curYaw));
          _yaw = this->normalize_heading(curYaw + dTheta);
     }
     if(isnan(_dx)) _dx = 0.0;
     if(isnan(_dy)) _dy = 0.0;
     // printf("linDist = %.3f, dTheta = %.3f, _dx = %.3f, _dy = %.3f, x = %.3f, y = %.3f, _yaw = %.3f\r\n", linDist, dTheta, _dx, _dy, curX, curY, _yaw);

     // Compute body velocities
     float lin_vel, ang_vel;
     if(fabs(dt) < 0.000001){
          lin_vel = 0.0;
          ang_vel = 0.0;
     } else{
          lin_vel = linDist / dt;
          ang_vel = dTheta / dt;
     }

     // Update current 2D location and velocities
     this->_current_pose[0] = curX + _dx;
     this->_current_pose[1] = curY + _dy;
     this->_current_pose[2] = _yaw;
     this->_linear_vel = lin_vel;
     this->_angular_vel = ang_vel;
     this->dist_traveled = linDist;
     this->dx = _dx;
     this->dy = _dy;
     this->dyaw = dTheta;

     // printf("Motor Speeds (m/s)/[PPS]:  %.3f / (%d)  | %.3f / (%d)  | %.3f / (%d)  | %.3f / (%d)\r\n",speeds[0],_speeds[0],speeds[1],_speeds[1],speeds[2],_speeds[2],speeds[3],_speeds[3]);
     // printf("Encoder Positions (qpps): %d | %d | %d | %d\r\n",tmpPos[0],tmpPos[1],tmpPos[2],tmpPos[3]);
     // printf("Δdistance, ΔX, ΔY, ΔYaw: %.3f, %.3f, %.3f, %.3f\r\n",dist_traveled, dx,dy,dyaw);
     // printf("Current Pose [X (m), Y (m), Yaw (rad)]: %.3f     |    %.3f   |       %.3f\r\n",_current_pose[0],_current_pose[1],_current_pose[5]);
}

void DualClaw::reset_encoders(){
     for(int i = 0; i <= 3;i++){ this->_last_positions[i] = 0; }

     this->leftclaw->ResetEncoders();
     this->rightclaw->ResetEncoders();

     this->leftclaw->ReadEncoders(this->_positions[0],this->_positions[1]);
     this->rightclaw->ReadEncoders(this->_positions[2],this->_positions[3]);

     this->_current_pose[0] = 0.0;
     this->_current_pose[1] = 0.0;
     this->_current_pose[2] = 0.0;

     printf("Encoders Reset (qpps): %d | %d | %d | %d\r\n",this->_positions[0],this->_positions[1],this->_positions[2],this->_positions[3]);
}

float DualClaw::normalize_heading(const float& angle){
     float tmpAngle = angle;
     while(tmpAngle > M_PI){ tmpAngle -= 2.0 * M_PI; }
     while(tmpAngle < -M_PI){ tmpAngle += 2.0 * M_PI; }
     return tmpAngle;
}
