#include <string.h>
#include <pigpiod_if2.h>

#include "robots/swansonV2.h"
#include "utilities/utils.h"
#include "base/definitions.h"

using namespace std;
using namespace chrono;

SwansonV2::SwansonV2(int pi){
     // Declare constants
	_count = 0;
     _rc_msg_count = 0;
     _header_count = 0;
	_gimbal_msg_count = 0;

     /**************************************************************************
     * LOAD CONFIG FILE
     **************************************************************************/
     // std::map<std::string, float> variables;
     // LoadInitialVariables("../../config/profiles/swanson.config", variables);

     // printf("%d %d %d %d %d %d %d %d\r\n",fr_pwm,fr_dir,fl_pwm,fl_dir,rr_pwm,rr_dir,rl_pwm,rl_dir);

     /**************************************************************************
     * END LOAD CONFIG FILE
     **************************************************************************/

     this->_pi = pi;
     this->_port = 14500;
     this->_max_speed = 1.5;
     this->_max_omega = this->_max_speed / 0.381;
	this->timeoutCnt = 0;

     string path = "/home/pi/devel/robo-commander/config/sensors";
     string file = "mpu9250";
     string datalog_file = "datalog";

     current_system_time = previous_system_time = start_system_time = system_clock::now();
	// open_datalog(datalog_file);

	this->mRelay = new AndroidAppInterface(this->_port);
	this->mRelay->mUdp->set_verbose(0);
     claws = new DualClaw(pi,"/home/pi/devel/robo-commander/config/profiles/dualclaw.config");
     imu = new GenericRTIMU(path, file);

     claws->set_odom_turn_direction(-1);
     // claws->flip_command_turn_direction();
     claws->reset_encoders();

     this->flag_verbose = false;
}

SwansonV2::~SwansonV2(){
     printf("SwansonV2 Shutting Down...\r\n");
     close_datalog();
     delete this->mRelay;
     delete claws;
     delete imu;
}

void SwansonV2::drive(float v, float w){
     vector<int32_t> cmds = claws->get_target_speeds(v, w);
     claws->drive(cmds);
}

void SwansonV2::update_sensors(){
     imu->update(5.0);
     claws->update_status();
     claws->update_odometry();
}

void SwansonV2::update_control_interface(){
	int err = this->mRelay->receiveUdpMessage();
	if(err < 0){
		this->timeoutCnt++;
		// printf("[INFO] AndroidAppInterface_Test() --- Timeout Count = %d\r\n",this->timeoutCnt);
	}
	// printf("[INFO] face->receiveUdpMessage() --- Returned code \'%d\'\r\n",err);
	AndroidInterfaceData tmpData = this->mRelay->getReceivedData();
	float v = tmpData.normalized_speed * this->_max_speed;
	float w = tmpData.normalized_turn_rate * this->_max_omega;
	// if((v != 0) || (w != 0)) printf("Velocity = %.3f, %.3f\r\n",v,w);

	this->cmdData.normalized_speed = v;
	this->cmdData.normalized_turn_rate = w;

	this->cmdData.front_cam_angle = tmpData.front_cam_angle;
	this->cmdData.left_cam_angle = tmpData.left_cam_angle;
	this->cmdData.right_cam_angle = tmpData.right_cam_angle;
	this->cmdData.back_cam_angle = tmpData.back_cam_angle;
}

vector<float> SwansonV2::get_sensor_data(){

     float roll, pitch, yaw, imu_dt;
     vector<float> raw_data;
     raw_data.reserve(NStates);

     // Perform Timestamping procedures
     current_system_time = system_clock::now();
     _time = duration_cast<duration<float>>(current_system_time-previous_system_time).count();

     // Grab Stored Data
     imu_dt = imu->get_update_period();
     vector<float> imu_data = imu->get_all_data();
     vector<float> encoder_data = claws->get_odom_deltas();

     // Reformat into single data vector
     raw_data.push_back(_time);
     raw_data.insert(raw_data.end(), imu_data.begin(), imu_data.end());
     raw_data.insert(raw_data.end(), encoder_data.begin(), encoder_data.end());

     // Miscellaneous Debugging
	// cout << "Swanson Sensor Data: ";
	// for(int i = 0;i<raw_data.size();i++){
     //      if(i==raw_data.size()) cout << raw_data.at(i);
     //      else cout << raw_data.at(i) << ",";
     // }
	// cout << endl;
     return raw_data;
}

void SwansonV2::open_datalog(string file_path){
	try{
		datalog.open(file_path + ".csv");
	     // time,ax,ay,az,gx,gy,gz,mx,my,mz,roll,pitch,yaw,od,oyaw,ox,oy;
	     datalog << "Time (sec), Accel X (m/s^2), Accel Y (m/s^2), Accel Z (m/s^2), Gyro X (rad/s), Gyro Y (rad/s), Gyro Z (rad/s), Mag X (μT), Mag Y (μT), Mag Z (μT), Quat X, Quat Y, Quat Z, Quat W, Roll (deg), Pitch (deg), Yaw (deg), Δdistance (m), ΔYaw (rad), ΔX (m), ΔY (m)\n";
	}
	catch(exception& e){
		printf("[ERROR] SwansonV2::open_datalog() ---- Could not open datalog file at \'%s\'\r\n",file_path.c_str());
		// cout << "Standard exception: " << e.what() << endl;
	}
}

void SwansonV2::close_datalog(){datalog.close();}

void SwansonV2::add_datalog_entry(vector<float> data){
     int n = data.size();
     for(int i = 0;i<n;i++){
          if(i==n)
               datalog << data.at(i);
          else
               datalog << data.at(i) << ",";
     }
     datalog << endl;
}
