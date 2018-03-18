#include <pigpiod_if2.h>
#include <string.h>

#include "robots/swansonV2.h"
#include "base/definitions.h"

using namespace std;
using namespace chrono;

SwansonV2::SwansonV2(int pi){

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

     string path = "/home/hunter/devel/robo-dev/config/sensors";
	string file = "mpu9250";
     string datalog_file = "datalog";

     current_system_time = previous_system_time = start_system_time = system_clock::now();
     open_datalog(datalog_file);

     rc_in = new UDP(_port,NULL);
     claws = new DualClaw(pi);
     imu = new IMU(path, file);

     claws->set_turn_direction(-1);
     claws->reset_encoders();

}

SwansonV2::~SwansonV2(){

     close_datalog();
     delete rc_in;
     delete claws;
     delete imu;

}


void SwansonV2::drive(float v, float w){

     vector<int32_t> cmds = claws->set_speeds(v, w);
     claws->drive(cmds);

}

void SwansonV2::readRC(){
     int i = 0;

     UDP* udp_line = rc_in;
     RC_COMMAND_MSG* data = &controls;

	char* dat = udp_line->read(sizeof(data)+24);
	memcpy(data, &dat[16],sizeof(data)+8);
}


void SwansonV2::updateSensors(){

     float roll,pitch,yaw;
     vector<float> log_entry(17);

     imu->update();
     vector<float> imu_data = imu->get_raw_data();
     // float imu_dt = imu->get_update_period();

     claws->update_status();
     claws->update_encoders();
     vector<float> encoder_data = claws->get_odom_deltas();

     /** Datalogging Section ONLY */
     current_system_time = system_clock::now();
     _time = duration_cast<duration<float>>(current_system_time-previous_system_time).count();

     log_entry.push_back(_time);
     log_entry.insert(log_entry.end(), imu_data.begin(), imu_data.end());
     log_entry.insert(log_entry.end(), encoder_data.begin(), encoder_data.end());


     roll = fmod((imu->euler[0]*M_RAD2DEG + 360.0),360.0);
     pitch = fmod((imu->euler[1]*M_RAD2DEG + 360.0),360.0);
     yaw = fmod((imu->euler[2]*M_RAD2DEG + 360.0),360.0);

     // printf("Current Pose [X (m), Y (m), Yaw (rad)]: %.3f     |    %.3f   |       %.3f\r\n",_current_pose[0],_current_pose[1],_current_pose[5]);
     // printf("V1, V2, V3, V4:     %.5f   |    %.5f |    %.5f |    %.5f \r\n",spd1,spd2,spd3,spd4);

}

void SwansonV2::open_datalog(string file_path){
     datalog.open(file_path + ".csv");
     //time,ax,ay,az,gx,gy,gz,mx,my,mz,roll,pitch,yaw,od,oyaw,ox,oy;
     datalog << "Time (sec), Accel X (m/s^2), Accel Y (m/s^2), Accel Z (m/s^2), Gyro X (rad/s), Gyro Y (rad/s), Gyro Z (rad/s), Mag X (μT), Mag Y (μT), Mag Z (μT), Roll (deg), Pitch (deg), Yaw (deg), Δdistance (m), ΔYaw (rad), ΔX (m), ΔY (m)\n";
}

void SwansonV2::close_datalog(){datalog.close();}

void SwansonV2::add_datalog_entry(vector<float> data){
     int n = data.size();
     for(int i = 0;i<=n;i++){
          if(i==n)
               datalog << data.at(i);
          else
               datalog << data.at(i) << ",";
     }
     datalog << endl;
}
