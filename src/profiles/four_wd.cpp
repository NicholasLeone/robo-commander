#include <thread>
#include <pigpiod_if2.h>

#include "utils/utils.h"
#include "four_wd.h"

using namespace std;

void dummyBack(int pos){
     // printf("%d\n", pos);
}

FourWD::FourWD(int pi){

     this->_pi = pi;
     this->_port = 14500;
     int encMode = ENCODER_MODE_STEP;

     /**************************************************************************
     * LOAD CONFIG FILE
     **************************************************************************/
     std::map<std::string, float> variables;
     LoadInitialVariables("config.txt", variables);

     int fr_pwm = (int) variables["fr_pwm"];
     int fr_dir = (int) variables["fr_dir"];
     int fl_pwm = (int) variables["fl_pwm"];
     int fl_dir = (int) variables["fl_dir"];
     int rr_pwm = (int) variables["rr_pwm"];
     int rr_dir = (int) variables["rr_dir"];
     int rl_pwm = (int) variables["rl_pwm"];
     int rl_dir = (int) variables["rl_dir"];

     int fr_a = (int) variables["fr_a"];
     int fr_b = (int) variables["fr_b"];
     int fl_a = (int) variables["fl_a"];
     int fl_b = (int) variables["fl_b"];
     int rr_a = (int) variables["rr_a"];
     int rr_b = (int) variables["rr_b"];
     int rl_a = (int) variables["rl_a"];
     int rl_b = (int) variables["rl_b"];

     int debounce = (int) variables["debounce"];
     int ppr = (int) variables["PPR"];
     int ratio = (int) variables["gears"];
     int baud = (int) variables["baud"];
     // char* ser_path = (string) variables["dev_path"];//ser_path.c_str()

     _mode = (int) variables["mode"];
     centerToWheelRadius = variables["centerToWheelRadius"];
     max_speed = variables["maxSpeed"];

     // printf("%d %d %d %d %d %d %d %d\r\n",fr_pwm,fr_dir,fl_pwm,fl_dir,rr_pwm,rr_dir,rl_pwm,rl_dir);

     /**************************************************************************
     * END LOAD CONFIG FILE
     **************************************************************************/


     rc_in = new UDP(_port,NULL);
     ser_handle = serial_open(pi,"/dev/ttyS0",baud,0);

     leftclaw = new RoboClaw(pi,ser_handle,128);
     rightclaw = new RoboClaw(pi,ser_handle,129);

     FR = new DcMotor(_pi,fr_pwm,fr_dir);
     FL = new DcMotor(_pi,fl_pwm,fl_dir);
     RR = new DcMotor(_pi,rr_pwm,rr_dir);
     RL = new DcMotor(_pi,rl_pwm,rl_dir);

     FR_enc = new Encoder(pi,fr_a, fr_b, -1, ppr, ratio, debounce, encMode, dummyBack);
     FL_enc = new Encoder(pi, fl_a, fl_b, -1, ppr, ratio, debounce, encMode, dummyBack);
     RR_enc = new Encoder(pi, rr_a, rr_b, -1, ppr, ratio, debounce, encMode, dummyBack);
     RL_enc = new Encoder(pi, rl_a, rl_b, -1, ppr, ratio, debounce, encMode, dummyBack);

     pidParamsLeft.dt = 0.01;
     pidParamsLeft.max = 1;
     pidParamsLeft.min = -1;
     pidParamsLeft.pre_error = 0;
     pidParamsLeft.integral = 0;
     pidParamsLeft.Kp = 7.0;
     pidParamsLeft.Ki = 7.0;
     pidParamsLeft.Kd = 0.0;

     pidParamsRight = pidParamsLeft;

     pidLeft = new PID(pidParamsLeft);
     pidRight = new PID(pidParamsRight);

     thread frSpd(&FR_enc->calc_speed,FR_enc);
     thread rrSpd(&RR_enc->calc_speed,RR_enc);
     thread flSpd(&FL_enc->calc_speed,FL_enc);
     thread rlSpd(&RL_enc->calc_speed,RL_enc);

     frSpd.detach();
     rrSpd.detach();
     flSpd.detach();
     rlSpd.detach();

}


FourWD::~FourWD(){
     drive(0,0);

     delete rc_in;
     delete leftclaw;
     delete rightclaw;

     delete FR;
     delete FL;
     delete RR;
     delete RL;

     delete FR_enc;
     delete FL_enc;
     delete RR_enc;
     delete RL_enc;

     delete pidLeft;
     delete pidRight;

     int err = serial_close(_pi, ser_handle);
     usleep(1 * 10000000);
}


void FourWD::_driveLeft(float speed){
     float front, rear;

     if(_mode == 0){ // Manual Mode (no PID)
          // Left Motors
          FL->setSpeed(speed);
          RL->setSpeed(speed);
     }else if(_mode == 1){    // PID Speed Control Mode
          front = calculateControlSpeed(pidLeft, speed, measured_speed_front_left);
          rear = calculateControlSpeed(pidLeft, speed, measured_speed_rear_left);

          FL->setSpeed(front);
          RL->setSpeed(rear);
     }else if(_mode == 2){    // RoboClaw Control
          int32_t clawSpd = speed * max_speed;
          leftclaw->SpeedM1M2(clawSpd,clawSpd);
     }

}


void FourWD::_driveRight(float speed){
     float front, rear;

     if(_mode == 0){ // Manual Mode (no PID)
          // Right Motors
          FR->setSpeed(speed);
          RR->setSpeed(speed);
     }else if(_mode == 1){    // PID Speed Control Mode
          front = calculateControlSpeed(pidRight, speed, measured_speed_front_right);
          rear = calculateControlSpeed(pidRight, speed, measured_speed_rear_right);

          FR->setSpeed(front);
          RR->setSpeed(rear);
     }else if(_mode == 2){    // RoboClaw Control
          int32_t clawSpd = speed * max_speed;
          rightclaw->SpeedM1M2(clawSpd,clawSpd);
     }

}


void FourWD::drive(float v, float w){

     /**   Differential Drive Drive Equations     */
     float vLeft = v - w * centerToWheelRadius;
     float vRight = v + w * centerToWheelRadius;
     target_speed_left = vLeft;
     target_speed_right = vRight;

     int32_t leftClawSpd = vLeft * max_speed;
     int32_t rightClawSpd = vRight * max_speed;

     _driveLeft(vLeft);
     _driveRight(vRight);

     // printf("V, Omega, Left, Right:     %.5f   |    %.5f |    %.5f |    %.5f \r\n",v,w,vLeft,vRight);

}

void FourWD::readRC(){
     int i = 0;

     UDP* udp_line = rc_in;
     RC_COMMAND_MSG* data = &controls;

	char* dat = udp_line->read(sizeof(data)+20);
	memcpy(data, &dat[16],sizeof(data)+4);
}

float FourWD::calculateControlSpeed(PID* _pid, float target, float measurement){

	// Get current value
	float normalized_measurement = measurement / max_speed;

	// Update Control Inputs
	float curCtrl = _pid->calculate(target, normalized_measurement);

     return curCtrl;
}

void FourWD::updateSensors(){

     measured_speed_front_left = this->FL_enc->getSpeed(1);
     measured_speed_front_right = this->FR_enc->getSpeed(1);
     measured_speed_rear_left = this->RL_enc->getSpeed(1);
     measured_speed_rear_right = this->RR_enc->getSpeed(1);

     measured_pos_front_left = this->FL_enc->getPosition();
     measured_pos_front_right = this->FR_enc->getPosition();
     measured_pos_rear_left = this->RL_enc->getPosition();
     measured_pos_rear_right = this->RR_enc->getPosition();

     // printf("V1, V2, V3, V4:     %.5f   |    %.5f |    %.5f |    %.5f \r\n",spd1,spd2,spd3,spd4);


}
