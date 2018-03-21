#include <pigpiod_if2.h>
#include <string.h>

#include "robots/swansonV2.h"
#include "utils/utils.h"
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
     printf("SwansonV2 Shutting Down...\r\n");
     close_datalog();
     delete rc_in;
     delete claws;
     delete imu;
}


void SwansonV2::drive(float v, float w){
     vector<int32_t> cmds = claws->set_speeds(v, w);
     claws->drive(cmds);
}

void SwansonV2::read_udp_header(){
	UDP* udp_line = rc_in;
     Udp_Msg_Header* data = &udp_header;

	char* dat = udp_line->read(sizeof(Udp_Msg_Header));
	memcpy(data, &dat[0],sizeof(Udp_Msg_Header));
     print_udp_header(data);
}

void SwansonV2::read_udp_commands(){

     UDP* udp_line = rc_in;
     RC_COMMAND_MSG* data = &controls;

     char* dat = udp_line->read(sizeof(data)+24);
     memcpy(data, &dat[16],sizeof(data)+8);


     // CommunicationHeaderByte* tmpHead;
     //      char* dat;
     //      int ready = 0;
     //
     //      //printf("Data Size, &Data Size, *Data Size, Data/Data size: %d, %d, %d, %d\r\n",sizeof(data),sizeof(&data),sizeof(*data),sizeof(data)/sizeof(data[0]));
     //
     //      while(ready == 0){
     //           dat = udp_line->read(sizeof(*data)+sizeof(*header));
     //           tmpHead = (CommunicationHeaderByte*)&dat[0];
     //
     //           int data_type = tmpHead->data_type;
     //           // TODO: Add in functionality to check for specific sensor index (needed for multiple same type of sensors)
     //
     //           if(data_type == SIMULATOR_DATA_IMU)
     //                ready = 1;
     //           else
     //                ready = 0;
     //      }
     //
     //      header = (CommunicationHeaderByte*)&dat[0];
     //      data = (Sim_Msg_IMUData*)&dat[20];
     //
     //      // printImu(*data);

}


void SwansonV2::update_sensors(){
     imu->update();
     claws->update_status();
     claws->update_encoders();
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
     vector<float> imu_data = imu->get_raw_data();
     vector<float> encoder_data = claws->get_odom_deltas();

     // Reformat into single data vector
     raw_data.push_back(_time);
     raw_data.insert(raw_data.end(), imu_data.begin(), imu_data.end());
     raw_data.insert(raw_data.end(), encoder_data.begin(), encoder_data.end());

     // Miscellaneous Debugging
     roll = fmod((imu->euler[0]*M_RAD2DEG + 360.0),360.0);
     pitch = fmod((imu->euler[1]*M_RAD2DEG + 360.0),360.0);
     yaw = fmod((imu->euler[2]*M_RAD2DEG + 360.0),360.0);

     return raw_data;
}

void SwansonV2::open_datalog(string file_path){
     datalog.open(file_path + ".csv");
     //time,ax,ay,az,gx,gy,gz,mx,my,mz,roll,pitch,yaw,od,oyaw,ox,oy;
     datalog << "Time (sec), Accel X (m/s^2), Accel Y (m/s^2), Accel Z (m/s^2), Gyro X (rad/s), Gyro Y (rad/s), Gyro Z (rad/s), Mag X (μT), Mag Y (μT), Mag Z (μT), Roll (deg), Pitch (deg), Yaw (deg), Δdistance (m), ΔYaw (rad), ΔX (m), ΔY (m)\n";
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

void SwansonV2::print_udp_header(Udp_Msg_Header* header){

     int32_t header_byte, msg_type, data_type, measurement_type, measurement_length;

     header_byte = (int32_t) header->id / 1000000;
     msg_type = (int32_t) header->msg_type / 1000000;
     data_type = (int32_t) header->data_type / 1000000;
     measurement_type = (int32_t) header->measurement_type / 1000000;
     measurement_length = (int32_t) header->measurement_length / 1000000;

	printf("=========== UDP Packet Info     =================\r\n");
     printf("UDP Packet Header:\r\n");
     printf("  Header Byte: %d\r\n", header_byte);
     printf("  Message Type: %d\r\n", msg_type);
     printf("  Data Type: %d\r\n", data_type);
     printf("  Measurement Type: %d\r\n", measurement_type);
     printf("  Measurement Length: %d\r\n", measurement_length);

}
