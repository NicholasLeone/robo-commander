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

     float _ser_path = variables["dev"];
     char* ser_path = (char*) to_string(_ser_path).c_str();
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

     leftclaw = new RoboClaw(pi, _ser_handle, 128);
     rightclaw = new RoboClaw(pi, _ser_handle, 129);
}


DualClaw::~DualClaw(){

     vector<int32_t> cmds{0,0};
     drive(cmds);

     delete leftclaw;
     delete rightclaw;

     int err = serial_close(_pi, _ser_handle);
     usleep(1 * 10000000);
}

vector<int32_t> DualClaw::set_speeds(float v, float w){

     vector<int32_t> cmds(2);

     /**   Differential Drive Drive Equations     */
     float v_left = v - w * (_base_width / 2.0);
     float v_right = v + w * (_base_width / 2.0);

     float left_spd = v_left * _qpps_per_meter * flag_left_sign;
     float right_spd = v_right * _qpps_per_meter * flag_right_sign;

     // printf("Left Velocity, Right Velocity:     %.5f   |    %.5f \r\n",left_spd, right_spd);

     // TODO: Make adaptable to variable number of motors (also account for motor order)
     cmds[0] = (int32_t)(left_spd);
     cmds[1] = (int32_t)(right_spd);

     return cmds;

}

void DualClaw::drive(vector<int32_t> cmds){

     int n = cmds.size();

     leftclaw->SpeedM1M2(cmds.at(0),cmds.at(0));
     rightclaw->SpeedM1M2(cmds.at(1),cmds.at(1));

}

void DualClaw::update_status(){
     int err[8];

     _main_battery[0] = leftclaw->ReadMainBatteryVoltage();
     _main_battery[1] = rightclaw->ReadMainBatteryVoltage();

     main_battery[0] = (float) (_main_battery[0]) / 10.0;
     main_battery[1] = (float) (_main_battery[1]) / 10.0;

     err[1] = leftclaw->ReadCurrents(_currents[0], _currents[1]);
     err[1] = rightclaw->ReadCurrents(_currents[2], _currents[3]);

     currents[0] = (float) (_currents[0]) / 100.0;
     currents[1] = (float) (_currents[1]) / 100.0;
     currents[2] = (float) (_currents[2]) / 100.0;
     currents[3] = (float) (_currents[3]) / 100.0;

     error[0] = leftclaw->ReadError();
     error[1] = rightclaw->ReadError();

     // TODO: User-friendly handling of errors

}

vector<float> DualClaw::get_currents(){
     vector<float> tmp {currents[0],currents[1],currents[2],currents[3]};
     return tmp;
}

vector<float> DualClaw::get_voltages(){
     vector<float> tmp {main_battery[0],main_battery[1]};
     return tmp;
}

vector<float> DualClaw::get_encoder_positions(){
     vector<float> tmp {positions[0],positions[1],positions[2],positions[3]};
     return tmp;
}

vector<float> DualClaw::get_encoder_speeds(){
     vector<float> tmp {speeds[0],speeds[1],speeds[2],speeds[3]};
     return tmp;
}

void DualClaw::update_encoders(){

     int tmpPos[4] = {0,0,0,0};
     float tmpDist[4] = {0,0,0,0};
     float avg_dist[2] = {0,0};

     _speeds[0] = leftclaw->ReadSpeedM1();
     _speeds[1] = leftclaw->ReadSpeedM2();
     _speeds[2] = rightclaw->ReadSpeedM1();
     _speeds[3] = rightclaw->ReadSpeedM2();

     for(int i = 0; i <= 3;i++){
          speeds[i] = (float) (_speeds[i]) / _qpps_per_meter;
     }

     // printf("Motor Speeds (m/s)/[PPS]: %.3f / (%d)  | %.3f / (%d)  | %.3f / (%d)  | %.3f / (%d)\r\n",speeds[0],_speeds[0],speeds[1],_speeds[1],speeds[2],_speeds[2],speeds[3],_speeds[3]);

     leftclaw->ReadEncoders(_positions[0],_positions[1]);
     rightclaw->ReadEncoders(_positions[2],_positions[3]);

     tmpPos[0] = _positions[0] * flag_left_sign;
     tmpPos[1] = _positions[1] * flag_left_sign;
     tmpPos[2] = _positions[2] * flag_right_sign;
     tmpPos[3] = _positions[3] * flag_right_sign;

     for(int i = 0; i <= 3;i++){
          tmpDist[i] = (float) (tmpPos[i] - _last_positions[i]) / _qpps_per_meter;
          _last_positions[i] = tmpPos[i];
     }

     // printf("Encoder Positions (qpps): %d | %d | %d | %d\r\n",tmpPos[0],tmpPos[1],tmpPos[2],tmpPos[3]);

     avg_dist[0] = (tmpDist[0] + tmpDist[1]) / 2.0;
     avg_dist[1] = (tmpDist[2] + tmpDist[3]) / 2.0;

     dDistance = (avg_dist[0] + avg_dist[1]) / 2.0;
     dTheta = (avg_dist[1] - avg_dist[0]) / _base_width;

     // printf("Odometry Updates: %.3f  | %.3f\r\n",dDistance,dTheta);
}

void DualClaw::reset_encoders(){

     for(int i = 0; i <= 3;i++){
          _last_positions[i] = 0;
     }

     leftclaw->ResetEncoders();
     rightclaw->ResetEncoders();

     leftclaw->ReadEncoders(_positions[0],_positions[1]);
     rightclaw->ReadEncoders(_positions[2],_positions[3]);

     printf("Encoders Reset (qpps): %d | %d | %d | %d\r\n",_positions[0],_positions[1],_positions[2],_positions[3]);

}
