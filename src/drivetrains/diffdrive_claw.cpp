#include <math.h>
#include <pigpiod_if2.h>

#include "utilities/utils.h"
#include "drivetrains/diffdrive_roboclaw.h"

using namespace std;
using namespace chrono;

/** SECTION: Constructors / Deconstructors
*
*/
DiffDriveClaw::DiffDriveClaw(){
     this->_cmd_turn_dir_sign = 1;
     this->_odom_turn_dir_sign = 1;
     this->_left_dir = 1;
     this->_right_dir = 1;

     this->_base_width = 0.219;
     this->_max_speed = 2.0;

     this->_qpps_per_meter = 9596;
     this->_wheel_diameter = 0.1905;
     this->_prev_enc_time = high_resolution_clock::now();
}
//DiffDriveClaw::DiffDriveClaw(int pi){
//     this->_pi = pi;
//
//     this->_cmd_turn_dir_sign = 1;
//     this->_odom_turn_dir_sign = 1;
//     this->_left_dir = 1;
//     this->_right_dir = 1;
//
//     this->_base_width = 0.219;
//     this->_max_speed = 2.0;
//
//     this->_qpps_per_meter = 9596;
//     this->_wheel_diameter = 0.1905;
//     this->_prev_enc_time = high_resolution_clock::now();
//}
//DiffDriveClaw::DiffDriveClaw(int pi, const char* config_file){
//     this->_pi = pi;
//
//     this->_cmd_turn_dir_sign = 1;
//     this->_odom_turn_dir_sign = 1;
//     this->_left_dir = 1;
//     this->_right_dir = 1;
//
//     int err = this->init(config_file);
//     if(err < 0){
//          printf("[ERROR] DiffDriveClaw() --- Could not initialize communication with one or more RoboClaws. Error code = %d\r\n",err);
//          exit(0);
//     }
//     this->_prev_enc_time = high_resolution_clock::now();
//}
//DiffDriveClaw::DiffDriveClaw(const char* config_file){
//     int pi = pigpio_start(NULL, NULL);
//     if(pi < 0){
//          printf("[ERROR] DiffDriveClaw() --- Could not initialize pigpiod \r\n");
//          exit(0);
//     }
//     this->_pi = pi;
//
//     this->_cmd_turn_dir_sign = 1;
//     this->_odom_turn_dir_sign = 1;
//     this->_left_dir = 1;
//     this->_right_dir = 1;
//
//     int err = this->init(config_file);
//     if(err < 0){
//          printf("[ERROR] DiffDriveClaw() --- Could not initialize communication with one or more RoboClaws. Error code = %d\r\n",err);
//          exit(0);
//     }
//     this->_prev_enc_time = high_resolution_clock::now();
//}
DiffDriveClaw::~DiffDriveClaw(){
     int err;
     printf("[INFO] DiffDriveClaw() --- Shutting Down...\r\n");
     vector<int32_t> cmds{0,0};
     this->drive(cmds);
     /**delete this->leftclaw;
     delete this->rightclaw;**/
	delete this->claw;

     // for(int i = 0; i < this->_ser_handles.size(); i++){
     for(int h : this->_ser_handles){
          err = serial_close(this->_pi, h);
          usleep(0.1 * 10000000);
     }
}

/** SECTION: Device Startup Functions
*
*/
//int DiffDriveClaw::init(const char* config_file){
//     /**************************************************************************
//     * LOAD CONFIG FILE
//     **************************************************************************/
//     std::map<std::string, std::string> variables;
//     LoadStringVariables(config_file, variables);
//
//     string _ser_path = variables["dev"];
//     char* ser_path = (char*) _ser_path.c_str();
//     int baud = stoi(variables["baud"]);
//     this->_base_width = stof(variables["base_width"]);
//     this->_max_speed = stof(variables["max_speed"]);
//     this->_qpps_per_meter = stoi(variables["qpps_per_meter"]);
//     this->_wheel_diameter = stof(variables["wheel_diameter"]);
//
//     // TODO: Add more tune-able parameters
//
//     // printf("%d %d %d %d %d %d %d %d\r\n",fr_pwm,fr_dir,fl_pwm,fl_dir,rr_pwm,rr_dir,rl_pwm,rl_dir);
//     printf("ROBOCLAW SETTINGS: \r\n");
//     printf("       Device Address: %s\r\n", ser_path);
//     printf("       Claw Baud Rate: %d\r\n", baud);
//     printf("       Base Width: %.4f\r\n", this->_base_width);
//     printf("       Max Speed (m/s): %.3f\r\n", this->_max_speed);
//     printf("       QPPS per Meter: %d\r\n", this->_qpps_per_meter);
//     printf("       Wheel Diameter (m): %f\r\n", this->_wheel_diameter);
//     printf("\r\n");
//
//     /**************************************************************************
//     * END LOAD CONFIG FILE
//     **************************************************************************/
//
//     int h = serial_open(this->_pi, ser_path, baud, 0);
//     if(h < 0){
//          printf("[ERROR] Could not initialize pigpiod serial device %s using baud of %d\r\n",ser_path,baud);
//          return -1;
//     }
//     this->_ser_handles.push_back(h);
//
//     /**this->leftclaw = new RoboClaw(this->_pi, h, 128);
//     this->rightclaw = new RoboClaw(this->_pi, h, 129);**/
//
//	this->claw = new RoboClaw(this->_pi, h, 128);
//
//     float kp[4]; float ki[4]; float kd[4]; uint32_t qpps[4];
//     /**this->leftclaw->ReadM1VelocityPID(kp[0],ki[0],kd[0],qpps[0]);
//     this->leftclaw->ReadM2VelocityPID(kp[1],ki[1],kd[1],qpps[1]);
//     this->rightclaw->ReadM1VelocityPID(kp[2],ki[2],kd[2],qpps[2]);
//     this->rightclaw->ReadM2VelocityPID(kp[3],ki[3],kd[3],qpps[3]);**/
//	this->claw->ReadM1VelocityPID(kp[0],ki[0],kd[0],qpps[0]);
//     this->claw->ReadM2VelocityPID(kp[2],ki[2],kd[2],qpps[2]);
//
//     printf("Current Pose [X (m), Y (m), Yaw (rad)]: %.3f     |    %.3f   |       %.3f\r\n",this->_cur_pose[0],this->_cur_pose[1],this->_cur_pose[2]);
//     printf("[MOTOR 1]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[0],ki[0],kd[0],qpps[0]);
//     printf("[MOTOR 2]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[1],ki[1],kd[1],qpps[1]);
//     printf("[MOTOR 3]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[2],ki[2],kd[2],qpps[2]);
//     printf("[MOTOR 4]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[3],ki[3],kd[3],qpps[3]);
//     printf("\r\n");
//     return 0;
//}
//int DiffDriveClaw::init(const char* serial_device, int baud,int claw_addr){
//     int h = serial_open(this->_pi, const_cast <char*>(serial_device), baud, 0);
//     if(h < 0){
//          printf("[ERROR] Could not initialize pigpiod serial device %s using baud of %d\r\n",serial_device,baud);
//          return -1;
//     }
//     this->_ser_handles.push_back(h);
//
//     /**this->leftclaw = new RoboClaw(this->_pi, h, left_claw_addr);
//     this->rightclaw = new RoboClaw(this->_pi, h, right_claw_addr);**/
//
//	this->claw = new RoboClaw(this->_pi, h, claw_addr);
//
//     float kp[4]; float ki[4]; float kd[4]; uint32_t qpps[4];
//     /**this->leftclaw->ReadM1VelocityPID(kp[0],ki[0],kd[0],qpps[0]);
//     this->leftclaw->ReadM2VelocityPID(kp[1],ki[1],kd[1],qpps[1]);
//     this->rightclaw->ReadM1VelocityPID(kp[2],ki[2],kd[2],qpps[2]);
//     this->rightclaw->ReadM2VelocityPID(kp[3],ki[3],kd[3],qpps[3]);**/
//	this->claw->ReadM1VelocityPID(kp[0],ki[0],kd[0],qpps[0]);
//     this->claw->ReadM2VelocityPID(kp[2],ki[2],kd[2],qpps[2]);
//
//     printf("Current Pose [X (m), Y (m), Yaw (rad)]: %.3f     |    %.3f   |       %.3f\r\n",this->_cur_pose[0],this->_cur_pose[1],this->_cur_pose[2]);
//     printf("[MOTOR 1]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[0],ki[0],kd[0],qpps[0]);
//     printf("[MOTOR 2]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[1],ki[1],kd[1],qpps[1]);
//     printf("[MOTOR 3]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[2],ki[2],kd[2],qpps[2]);
//     printf("[MOTOR 4]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[3],ki[3],kd[3],qpps[3]);
//     printf("\r\n");
//     return 0;
//}
//int DiffDriveClaw::init(int baud, const char* serial_device, int claw_addr){
//     int err = 0;
//     /**int hleft = serial_open(this->_pi, const_cast <char*>(serial_device_left), baud, 0);
//     if(hleft < 0){
//          printf("[WARNING] DiffDriveClaw::init() --- pigpiod could not initialize left serial device \'%s\' using baud of %d\r\n",serial_device_left,baud);
//          err -= 1;
//     }
//     int hright = serial_open(this->_pi, const_cast <char*>(serial_device_right), baud, 0);
//     if(hright < 0){
//          printf("[WARNING] DiffDriveClaw::init() --- pigpiod could not initialize right serial device \'%s\' using baud of %d\r\n",serial_device_right,baud);
//          err -= 1;
//     }
//     if(err < 0){
//          printf("[ERROR] DiffDriveClaw::init() --- Unable to successfully establish communication with one, or more, Roboclaw devices, exiting.\r\n");
//          return err;
//     }
//     this->_ser_handles.push_back(hleft);
//     this->_ser_handles.push_back(hright);**/
//
//	int h = serial_open(this->_pi, const_cast <char*>(serial_device), baud, 0);
//
//	if(h < 0){
//          printf("[WARNING] DiffDriveClaw::init() --- pigpiod could not initialize serial device \'%s\' using baud of %d\r\n",serial_device,baud);
//          err -= 1;
//     }
//     if(err < 0){
//          printf("[ERROR] DiffDriveClaw::init() --- Unable to successfully establish communication with Roboclaw device, exiting.\r\n");
//          return err;
//     }
//
//	this->_ser_handles.push_back(h);
//
//
//     /**this->leftclaw = new RoboClaw(this->_pi, h, left_claw_addr);
//     this->rightclaw = new RoboClaw(this->_pi, h, right_claw_addr);**/
//
//	this->claw = new RoboClaw(this->_pi, h, claw_addr);
//
//     float kp[4]; float ki[4]; float kd[4]; uint32_t qpps[4];
//     /**this->leftclaw->ReadM1VelocityPID(kp[0],ki[0],kd[0],qpps[0]);
//     this->leftclaw->ReadM2VelocityPID(kp[1],ki[1],kd[1],qpps[1]);
//     this->rightclaw->ReadM1VelocityPID(kp[2],ki[2],kd[2],qpps[2]);
//     this->rightclaw->ReadM2VelocityPID(kp[3],ki[3],kd[3],qpps[3]);**/
//	this->claw->ReadM1VelocityPID(kp[0],ki[0],kd[0],qpps[0]);
//     this->claw->ReadM2VelocityPID(kp[2],ki[2],kd[2],qpps[2]);
//
//     printf("Current Pose [X (m), Y (m), Yaw (rad)]: %.3f     |    %.3f   |       %.3f\r\n",this->_cur_pose[0],this->_cur_pose[1],this->_cur_pose[2]);
//     printf("[MOTOR 1]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[0],ki[0],kd[0],qpps[0]);
//     printf("[MOTOR 2]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[1],ki[1],kd[1],qpps[1]);
//     printf("[MOTOR 3]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[2],ki[2],kd[2],qpps[2]);
//     printf("[MOTOR 4]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[3],ki[3],kd[3],qpps[3]);
//     printf("\r\n");
//     return 0;
//}

int DiffDriveClaw::init(bool doack, const char* port, uint8_t claw_addr, uint32_t timeout) {
    this->claw = new RoboclawUnix(port, claw_addr, timeout, doack);

    float kp[4]; float ki[4]; float kd[4]; uint32_t qpps[4];
    /**this->leftclaw->ReadM1VelocityPID(kp[0],ki[0],kd[0],qpps[0]);
    this->leftclaw->ReadM2VelocityPID(kp[1],ki[1],kd[1],qpps[1]);
    this->rightclaw->ReadM1VelocityPID(kp[2],ki[2],kd[2],qpps[2]);
    this->rightclaw->ReadM2VelocityPID(kp[3],ki[3],kd[3],qpps[3]);**/
    this->claw->ReadM1VelocityPID(kp[0],ki[0],kd[0],qpps[0]);
    this->claw->ReadM2VelocityPID(kp[2],ki[2],kd[2],qpps[2]);

    printf("Current Pose [X (m), Y (m), Yaw (rad)]: %.3f     |    %.3f   |       %.3f\r\n",this->_cur_pose[0],this->_cur_pose[1],this->_cur_pose[2]);
    printf("[MOTOR 1]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[0],ki[0],kd[0],qpps[0]);
    printf("[MOTOR 2]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[1],ki[1],kd[1],qpps[1]);
    printf("[MOTOR 3]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[2],ki[2],kd[2],qpps[2]);
    printf("[MOTOR 4]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[3],ki[3],kd[3],qpps[3]);
    printf("\r\n");
    return 0;
}

int DiffDriveClaw::init(const char* port, bool doack, uint8_t claw_addr, uint32_t timeout) {
    this->claw = new RoboclawUnix(port, claw_addr, timeout, doack);

    float kp[4]; float ki[4]; float kd[4]; uint32_t qpps[4];
    /**this->leftclaw->ReadM1VelocityPID(kp[0],ki[0],kd[0],qpps[0]);
    this->leftclaw->ReadM2VelocityPID(kp[1],ki[1],kd[1],qpps[1]);
    this->rightclaw->ReadM1VelocityPID(kp[2],ki[2],kd[2],qpps[2]);
    this->rightclaw->ReadM2VelocityPID(kp[3],ki[3],kd[3],qpps[3]);**/
    this->claw->ReadM1VelocityPID(kp[0],ki[0],kd[0],qpps[0]);
    this->claw->ReadM2VelocityPID(kp[2],ki[2],kd[2],qpps[2]);

    printf("Current Pose [X (m), Y (m), Yaw (rad)]: %.3f     |    %.3f   |       %.3f\r\n",this->_cur_pose[0],this->_cur_pose[1],this->_cur_pose[2]);
    printf("[MOTOR 1]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[0],ki[0],kd[0],qpps[0]);
    printf("[MOTOR 2]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[1],ki[1],kd[1],qpps[1]);
    printf("[MOTOR 3]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[2],ki[2],kd[2],qpps[2]);
    printf("[MOTOR 4]   KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp[3],ki[3],kd[3],qpps[3]);
    printf("\r\n");
    return 0;
}

/** SECTION: Device Motion Control Functions
*
*/
vector<int32_t> DiffDriveClaw::get_target_speeds(float v, float w){
     if(fabs(v) > this->_max_speed) v= this->_max_speed;
     vector<int32_t> cmds(2);

     /**   Differential Drive Drive Equations     */
     float v_left = v - (this->_cmd_turn_dir_sign * w) * (this->_base_width / 2.0);
     float v_right = v + (this->_cmd_turn_dir_sign * w) * (this->_base_width / 2.0);

     float left_spd = v_left * this->_qpps_per_meter * this->_left_dir;
     float right_spd = v_right * this->_qpps_per_meter * this->_right_dir;

     // printf("Left Velocity, Right Velocity:     %.5f   |    %.5f \r\n",left_spd, right_spd);

     // TODO: Make adaptable to variable number of motors (also account for motor order)
     cmds[0] = (int32_t)(left_spd);
     cmds[1] = (int32_t)(right_spd);

     return cmds;
}
void DiffDriveClaw::drive(vector<int32_t> cmds){
     int n = cmds.size();
     /**this->leftclaw->SpeedM1M2(cmds.at(0),cmds.at(0));
     this->rightclaw->SpeedM1M2(cmds.at(1),cmds.at(1));**/
	this->claw->SpeedM1M2(cmds.at(0), cmds.at(1));
}
void DiffDriveClaw::drive(float v, float w){
     vector<int32_t> cmds = this->get_target_speeds(v,w);
     int n = cmds.size();
	/**this->leftclaw->SpeedM1M2(cmds.at(0),cmds.at(0));
     this->rightclaw->SpeedM1M2(cmds.at(1),cmds.at(1));**/
	this->claw->SpeedM1M2(cmds.at(0), cmds.at(1));
}

/** SECTION: Device Data Request/Update Functions
*
*/
void DiffDriveClaw::update_status(bool verbose){
     bool success;
     /**bool valid1,valid2,valid3,valid4;
     int16_t leftAmps1, leftAmps2, rightAmps1, rightAmps2;**/
	bool valid1, valid2;
	int16_t leftAmp, rightAmp;

     /** Query roboclaws for information and store for later */
     /**uint16_t leftVolt = this->leftclaw->ReadMainBatteryVoltage(&valid1);
     uint16_t rightVolt = this->rightclaw->ReadMainBatteryVoltage(&valid2);**/
	uint16_t volt = this->claw->ReadMainBatteryVoltage(&valid1);
     /** Convert recevied information into user-friendly format */
     /**float leftBatLvl, rightBatLvl;
     if(valid1) leftBatLvl = (float) ((int16_t) leftVolt) / 10.0;
     else leftBatLvl = -1.0;
     if(valid2) rightBatLvl = (float) ((int16_t) rightVolt) / 10.0;
     else rightBatLvl = -1.0;**/
	float batLvl;
	if(valid1) batLvl = (float) ((int16_t) volt) / 10.0;
     /** Package converted data containers */
     /**vector<float> voltVec{leftBatLvl, rightBatLvl};**/

     /** Query roboclaws for information and store for later */
     /**success = this->leftclaw->ReadCurrents(leftAmps1, leftAmps2);
     if(!success){
          leftAmps1 = -100;
          leftAmps2 = -100;
     }
     success = this->rightclaw->ReadCurrents(rightAmps1, rightAmps2);
     if(!success){
          rightAmps1 = -100;
          rightAmps2 = -100;
     }**/
	success = this->claw->ReadCurrents(leftAmp, rightAmp);
	if(!success) {
		leftAmp = -100;
		rightAmp = -100;
	}
     /** Convert recevied information into user-friendly format */
     /**float leftCurrent1 = ( (float) leftAmps1 ) / 100.0;
     float leftCurrent2 = ( (float) leftAmps2 ) / 100.0;
     float rightCurrent1 = ( (float) rightAmps1 ) / 100.0;
     float rightCurrent2 = ( (float) rightAmps2 ) / 100.0;**/
	float leftCurrent = ( (float) leftAmp) / 100.0;
	float rightCurrent = ( (float) rightAmp) / 100.0;
     /** Package converted data containers */
     /**vector<float> currentVec{leftCurrent1, leftCurrent2, rightCurrent1, rightCurrent2};**/
	vector<float> currentVec{leftCurrent, rightCurrent};

     /** Query roboclaws for information and store for later */
     // uint16_t leftErrStatus = this->leftclaw->ReadError(&valid1);
     // uint16_t rightErrStatus = this->rightclaw->ReadError(&valid2);
     // vector<uint16_t> errVec{leftErrStatus, rightErrStatus};

     /**if(verbose){
          printf("Battery Voltages:     %.3f |    %.3f\r\n",leftBatLvl, rightBatLvl);
          printf("Motor Currents:     %.3f |    %.3f |    %.3f |    %.3f\r\n", leftCurrent1, leftCurrent2, rightCurrent1, rightCurrent2);
          // printf("Motor Currents:     %d |    %d |    %d |    %d\r\n", leftAmps1, leftAmps2, rightAmps1, rightAmps2);
     }**/
	if(verbose) {
         printf("Battery Voltage:     %.3f \r\n", batLvl);
         printf("Motor Currents:     %.3f |    %.3f\r\n", leftCurrent, rightCurrent);
         /** Store received data internally */
         this->_lock.lock();
         // this->_claw_error_status = vector<uint16_t>(errVec.begin(), errVec.end());
         this->_main_battery_voltage = batLvl;
         this->_motor_currents = vector<float>(currentVec.begin(), currentVec.end());
         this->_lock.unlock();
    }
}
void DiffDriveClaw::update_motors(bool verbose){
     bool success;
     /**bool valid1, valid2, valid3, valid4;
     uint8_t status1, status2, status3, status4;**/
	bool valid1, valid2;
	uint8_t status1, status2;
     /** Query roboclaws for information related to current motor velocities first */
     /**uint32_t leftM1Pps = this->leftclaw->ReadSpeedM1(&status1,&valid1);
     uint32_t leftM2Pps = this->leftclaw->ReadSpeedM2(&status2,&valid2);
     uint32_t rightM1Pps = this->rightclaw->ReadSpeedM1(&status3,&valid3);
     uint32_t rightM2Pps = this->rightclaw->ReadSpeedM2(&status4,&valid4);**/
	uint32_t leftMPps = this->claw->ReadSpeedM1(&status1, &valid1);
	uint32_t rightMPps = this->claw->ReadSpeedM2(&status2, &valid2);
     /** Convert motor motion information into user-friendly format */
     /**float leftM1Vel = (float) ((int32_t) leftM1Pps) / (float) this->_qpps_per_meter;
     float leftM2Vel = (float) ((int32_t) leftM2Pps) / (float) this->_qpps_per_meter;
     float rightM1Vel = (float) ((int32_t) rightM1Pps) / (float) this->_qpps_per_meter;
     float rightM2Vel = (float) ((int32_t) rightM2Pps) / (float) this->_qpps_per_meter;**/

	float leftMVel = (float) ((int32_t) leftMPps) / (float) this->_qpps_per_meter;
	float rightMVel = (float) ((int32_t) rightMPps) / (float) this->_qpps_per_meter;

     /** Query encoders information to be used for deriving odometry change */
     /** uint32_t leftPosM1, leftPosM2, rightPosM1, rightPosM2;*/
     uint32_t leftPosM, rightPosM;
     /**success = this->leftclaw->ReadEncoders(leftPosM1, leftPosM2);
     success = this->rightclaw->ReadEncoders(rightPosM1, rightPosM2);**/
     success = this->claw->ReadEncoders(leftPosM, rightPosM);

     /** Get elapsed time since last encoder update and store internally */
     high_resolution_clock::time_point now = high_resolution_clock::now();
     duration<float> time_span = duration_cast<duration<float>>(now - this->_prev_enc_time);

     /** Package extracted data into containers */
     vector<uint32_t> encVec{leftPosM, rightPosM};
     vector<uint32_t> ppsVec{leftMPps, rightMPps};
     vector<float> spdVec{leftMVel, rightMVel};
     /** Store current motor information internally */
     this->_lock.lock();
     this->_sampled_enc_dt = time_span.count();
     this->_prev_enc_time = now;
     this->_motors_pps = vector<uint32_t>(ppsVec.begin(), ppsVec.end());
     this->_motor_velocities = vector<float>(spdVec.begin(), spdVec.end());
     this->_encoder_positions = vector<uint32_t>(encVec.begin(), encVec.end());
     this->_lock.unlock();
}
void DiffDriveClaw::update_odometry(bool verbose){
     this->update_motors();
     /**float encoderDeltas[4] = {0.0,0.0,0.0,0.0};**/
     float encoderDeltas[2] = {0.0, 0.0};
     this->_lock.lock();
     for(int i = 0; i < 2; i++){
          int32_t curEncPos = (int32_t) this->_encoder_positions[i];
          int32_t prevEncPos = (int32_t) this->_prev_encoder_positions[i];
          encoderDeltas[i] =  (float) (curEncPos - prevEncPos) / (float) this->_qpps_per_meter;
          this->_prev_encoder_positions[i] = (uint32_t) curEncPos;
     }
     // Store these values locally before unlock
     float curPose[3] = {this->_cur_pose[0], this->_cur_pose[1], this->_cur_pose[2]};
     float encDt = this->_sampled_enc_dt;
     this->_lock.unlock();

     /** Compute odometry and pose changes based on encoder deltas */
     /**float avgDistL = (encoderDeltas[0] + encoderDeltas[1]) / 2.0;
     float avgDistR = (encoderDeltas[2] + encoderDeltas[3]) / 2.0;**/
     float linDistTraveled = (encoderDeltas[0] + encoderDeltas[1]) / 2.0;
     float dTheta = (encoderDeltas[1] - encoderDeltas[0]) / this->_base_width;
     dTheta = (float)(this->_odom_turn_dir_sign) * dTheta;

     // Prevent wrong odometry from any potential data corruption
     if(isnan(linDistTraveled)) linDistTraveled = 0.0;
     float curX, curY, curYaw;
     if(isnan(curPose[0])) curX = 0.0;
     else curX = curPose[0];
     if(isnan(curPose[1])) curY = 0.0;
     else curY = curPose[1];
     if(isnan(curPose[2])) curYaw = 0.0;
     else curYaw = curPose[2];

     float dX, dY, dYaw;
     if(isnan(dTheta)){
          dTheta = 0.0;
          dX = linDistTraveled * cos(curYaw);
          dY = linDistTraveled * sin(curYaw);
          dYaw = 0;
     } else{
//          float r = linDistTraveled / dTheta;
//          dX = r * (sin(curYaw + dTheta) - sin(curYaw));
//          dY = -r * (cos(curYaw + dTheta) - cos(curYaw));
          dX = linDistTraveled * cos(curYaw + (dTheta / 2));
          dY = linDistTraveled * sin(curYaw + (dTheta / 2));
//          dYaw = this->normalize_heading(curYaw + dTheta);
         dYaw = this->normalize_heading(dTheta);
     }
     // Prevent wrong odometry from any potential data corruption
     if(isnan(dX)) dX = 0.0;
     if(isnan(dY)) dY = 0.0;

     // Compute body velocities
     float linVel, angVel;
     if(fabs(encDt) < 0.000001){
          linVel = 0.0;
          angVel = 0.0;
     } else{
          linVel = linDistTraveled / encDt;
          angVel = dTheta / encDt;
     }

     if(verbose){
          // printf("Motor Speeds (m/s)/[PPS]:  %.3f / (%d)  | %.3f / (%d)  | %.3f / (%d)  | %.3f / (%d)\r\n",speeds[0],_speeds[0],speeds[1],_speeds[1],speeds[2],_speeds[2],speeds[3],_speeds[3]);
          // printf("Encoder Positions (qpps): %d | %d | %d | %d\r\n",tmpPos[0],tmpPos[1],tmpPos[2],tmpPos[3]);
          // printf("Δdistance, ΔX, ΔY, ΔYaw: %.3f, %.3f, %.3f, %.3f\r\n",dist_traveled, dx,dy,dyaw);
          // printf("Current Pose [X (m), Y (m), Yaw (rad)]: %.3f     |    %.3f   |       %.3f\r\n",_current_pose[0],_current_pose[1],_current_pose[5]);
     }

     // Update internally stored variables
     this->_lock.lock();
     this->_cur_pose[0] = curX + dX;
     this->_cur_pose[1] = curY + dY;
     this->_cur_pose[2] = curYaw + dYaw;
     this->_linear_vel = linVel;
     this->_angular_vel = angVel;
     this->_odom_changes[0] = linDistTraveled;
     this->_odom_changes[1] = dX;
     this->_odom_changes[2] = dY;
     this->_odom_changes[3] = dTheta;
     this->_lock.unlock();
}

/** SECTION: Class Config Getters / Setters
*
*/
void DiffDriveClaw::set_odom_turn_direction(int dir){ this->_odom_turn_dir_sign = dir; }
void DiffDriveClaw::set_command_turn_direction(int dir){ this->_cmd_turn_dir_sign = dir; }
void DiffDriveClaw::set_base_width(float width){ this->_base_width = width; }
void DiffDriveClaw::set_max_speed(float speed){ this->_max_speed = speed; }
void DiffDriveClaw::set_qpps_per_meter(int qpps){ this->_qpps_per_meter = qpps; }
void DiffDriveClaw::set_wheel_diameter(float diameter){ this->_wheel_diameter = diameter; }
float DiffDriveClaw::get_max_speed(){ return this->_max_speed; }
float DiffDriveClaw::get_base_width(){ return this->_base_width; }
int DiffDriveClaw::get_qpps_per_meter(){ return this->_qpps_per_meter; }
float DiffDriveClaw::get_wheel_diameter(){ return this->_wheel_diameter; }

/** SECTION: Class Stored Data Getters
*
*/
vector<float> DiffDriveClaw::get_currents(){
     vector<float> out;
     for(float val : this->_motor_currents){ out.push_back(val); }
     return out;
}
float DiffDriveClaw::get_voltage(){
     return _main_battery_voltage;
}
vector<uint16_t> DiffDriveClaw::get_error_status(){
     vector<uint16_t> out;
     for(uint16_t val : this->_claw_error_status){ out.push_back(val); }
     return out;
}
vector<uint32_t> DiffDriveClaw::get_encoder_positions(){
     vector<uint32_t> out;
     for(uint32_t val : this->_encoder_positions){ out.push_back(val); }
     return out;
}
vector<uint32_t> DiffDriveClaw::get_motor_pps(){
     vector<uint32_t> out;
     for(uint32_t val : this->_motors_pps){ out.push_back(val); }
     return out;
}
vector<float> DiffDriveClaw::get_motor_speeds(){
     vector<float> out;
     for(float val : this->_motor_velocities){ out.push_back(val); }
     return out;
}
vector<float> DiffDriveClaw::get_velocities(){
     vector<float> out{this->_linear_vel,this->_angular_vel};
     return out;
}
vector<float> DiffDriveClaw::get_odom_deltas(){
     vector<float> out{this->_odom_changes[0],this->_odom_changes[1],this->_odom_changes[2],this->_odom_changes[3]};
     return out;
}
vector<float> DiffDriveClaw::get_pose(){
     vector<float> out{this->_cur_pose[0],this->_cur_pose[1],this->_cur_pose[2]};
     return out;
}

/** SECTION: Helper Functions
*
*/
void DiffDriveClaw::reset_encoders(bool verbose){
     /**this->leftclaw->ResetEncoders();
     this->rightclaw->ResetEncoders();**/
     this->claw->ResetEncoders();
     // if(verbose) printf("Encoders Reset (qpps): %d | %d | %d | %d\r\n",this->_encoder_positions[0],this->_encoder_positions[1],this->_encoder_positions[2],this->_encoder_positions[3]);
}
void DiffDriveClaw::reset_odometry(bool verbose){
     this->_lock.lock();
     this->reset_encoders();
     for(int i = 0; i < 4; i++){ this->_prev_encoder_positions[i] = 0; }
     for(int i = 0; i < 4; i++){ this->_odom_changes[i] = 0.0; }
     for(int j = 0; j < 3; j++){ this->_cur_pose[j] = 0.0; }
     this->_lock.unlock();
     if(verbose) printf("Odometry Reset --- Current Pose (X, Y, Heading) = %.3f | %.3f | %.3f\r\n",this->_cur_pose[0],this->_cur_pose[1],this->_cur_pose[2]);
}
float DiffDriveClaw::normalize_heading(const float& angle){
     float tmpAngle = angle;
     while(tmpAngle > M_PI){ tmpAngle -= 2.0 * M_PI; }
     while(tmpAngle < -M_PI){ tmpAngle += 2.0 * M_PI; }
     return tmpAngle;
}
