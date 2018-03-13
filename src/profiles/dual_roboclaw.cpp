#include <thread>
#include <pigpiod_if2.h>

#include "utils/utils.h"
#include "dual_roboclaw.h"

using namespace std;

DualClaw::DualClaw(int pi){

     this->_pi = pi;

     flag_turn_dir = 1;
     flag_left_sign = 1;
     flag_right_sign = 1;

     /**************************************************************************
     * LOAD CONFIG FILE
     **************************************************************************/
     std::map<std::string, std::string> variables;
     LoadStringVariables("/home/hunter/devel/robo-dev/config/profiles/dualclaw.config", variables);

     string _ser_path = variables["dev"];
     char* ser_path = (char*) _ser_path.c_str();
     int baud = stoi(variables["baud"]);
     _base_width = stof(variables["base_width"]);
     _max_speed = stof(variables["max_speed"]);
     _qpps_per_meter = stoi(variables["qpps_per_meter"]);
     _wheel_diameter = (uint32_t) stof(variables["wheel_diameter"]);

     // TODO: Add more tune-able parameters

     // printf("%d %d %d %d %d %d %d %d\r\n",fr_pwm,fr_dir,fl_pwm,fl_dir,rr_pwm,rr_dir,rl_pwm,rl_dir);
     printf("ROBOCLAW SETTINGS: \r\n");
     printf("       Device Address: %s\r\n", ser_path);
     printf("       Claw Baud Rate: %d\r\n", baud);
     printf("       Base Width: %.4f\r\n", _base_width);
     printf("       Max Speed (m/s): %.3f\r\n", _max_speed);
     printf("       QPPS per Meter: %d\r\n", _qpps_per_meter);
     printf("\r\n");

     /**************************************************************************
     * END LOAD CONFIG FILE
     **************************************************************************/

     _ser_handle = serial_open(pi, ser_path, baud, 0);

     leftclaw = new RoboClaw(pi, _ser_handle, 128);
     rightclaw = new RoboClaw(pi, _ser_handle, 129);


     leftclaw->ReadM1VelocityPID(kp[0],ki[0],kd[0],qpps[0]);
     leftclaw->ReadM2VelocityPID(kp[1],ki[1],kd[1],qpps[1]);
     rightclaw->ReadM1VelocityPID(kp[2],ki[2],kd[2],qpps[2]);
     rightclaw->ReadM2VelocityPID(kp[3],ki[3],kd[3],qpps[3]);

     printf("[MOTOR 1]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[0],ki[0],kd[0],qpps[0]);
     printf("[MOTOR 2]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[1],ki[1],kd[1],qpps[1]);
     printf("[MOTOR 3]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[2],ki[2],kd[2],qpps[2]);
     printf("[MOTOR 4]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[3],ki[3],kd[3],qpps[3]);
     printf("\r\n");

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
     uint8_t status;
     bool valid1,valid2,valid3,valid4;

     _main_battery[0] = leftclaw->ReadMainBatteryVoltage(&valid1);
     _main_battery[1] = rightclaw->ReadMainBatteryVoltage(&valid2);

     main_battery[0] = (float) (_main_battery[0]) / 10.0;
     main_battery[1] = (float) (_main_battery[1]) / 10.0;

     err[1] = leftclaw->ReadCurrents(_currents[0], _currents[1]);
     err[1] = rightclaw->ReadCurrents(_currents[2], _currents[3]);

     currents[0] = (float) (_currents[0]) / 100.0;
     currents[1] = (float) (_currents[1]) / 100.0;
     currents[2] = (float) (_currents[2]) / 100.0;
     currents[3] = (float) (_currents[3]) / 100.0;

     error[0] = leftclaw->ReadError(&valid3);
     error[1] = rightclaw->ReadError(&valid4);

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

     uint8_t status1, status2, status3, status4;
     bool valid1, valid2, valid3, valid4;

     uint32_t tmpPos[4] = {0,0,0,0};
     float tmpDist[4] = {0,0,0,0};
     float avg_dist[2] = {0,0};

     _speeds[0] = leftclaw->ReadSpeedM1(&status1,&valid1);
     _speeds[1] = leftclaw->ReadSpeedM2(&status2,&valid2);
     _speeds[2] = rightclaw->ReadSpeedM1(&status3,&valid3);
     _speeds[3] = rightclaw->ReadSpeedM2(&status4,&valid4);

     for(int i = 0; i <= 3;i++){
          speeds[i] = (float) (_speeds[i] / _qpps_per_meter);
     }

     printf("Motor Speeds (m/s)/[PPS]: %.3f / (%d)  | %.3f / (%d)  | %.3f / (%d)  | %.3f / (%d)\r\n",speeds[0],_speeds[0],speeds[1],_speeds[1],speeds[2],_speeds[2],speeds[3],_speeds[3]);

     leftclaw->ReadEncoders(_positions[0],_positions[1]);
     rightclaw->ReadEncoders(_positions[2],_positions[3]);

     tmpPos[0] = _positions[0] * flag_left_sign;
     tmpPos[1] = _positions[1] * flag_left_sign;
     tmpPos[2] = _positions[2] * flag_right_sign;
     tmpPos[3] = _positions[3] * flag_right_sign;

     for(int i = 0; i <= 3;i++){
          tmpDist[i] = ((float) (tmpPos[i] - _last_positions[i])) / _qpps_per_meter;
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
