#include <pigpiod_if2.h>
#include <string.h>

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

     string path = "/home/hunter/devel/robo-dev/config/sensors";
     string file = "mpu9250";
     string datalog_file = "datalog";

     current_system_time = previous_system_time = start_system_time = system_clock::now();
     // open_datalog(datalog_file);

     rc_in = new UDP(_port,NULL);
     // claws = new DualClaw(pi);
     // imu = new IMU(path, file);
     //
     // claws->set_turn_direction(-1);
     // claws->reset_encoders();

     this->flag_verbose = false;
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

void SwansonV2::readHeader(char* msgBuffer){
     Sim_Msg_Data_Header* data = &_header;
     memcpy(data, &msgBuffer[0],sizeof(Sim_Msg_Data_Header));
     this->_header_count++;
}

void SwansonV2::readRC(char* msgBuffer){
     Sim_Msg_MotionCommands* data = &controls;
	memcpy(data, &msgBuffer[16],sizeof(*data));
	this->_rc_msg_count++;
}

void SwansonV2::readGimbalAngles(char* msgBuffer){
     /** Original code for RC_ROS_Bridge */
     // Sim_CameraGimbalsCommand* data = &angles;
	// memcpy(data, &msgBuffer[8],sizeof(*data));

     /** Experimental code working w/ Android app */
     int ang1 = (int)msgBuffer[8]; // R
     int ang2 = (int)msgBuffer[9]; // L
     int ang3 = (int)msgBuffer[10]; // B
     int ang4 = (int)msgBuffer[11]; // F
     this->angles.right_angle = ang1 - 90.0;
     this->angles.left_angle = ang2 - 90.0;
     this->angles.back_angle = ang3 - 90.0;
     this->angles.front_angle = ang4 - 90.0;

	this->_gimbal_msg_count++;
}

void SwansonV2::receiveUdpMessage(){
     char* msgBuf = nullptr;
     char* p = nullptr;
     int mtot = 0;
     int msize = 0;

     // rc_in->flush();
     msgBuf = this->rc_in->read(4096);
     int nBytes = this->rc_in->bytesRead;
     // printf("Received %d bytes over UDP\r\n",nBytes);
     do{
          p = &msgBuf[mtot];

          this->readHeader(p);
          // this->print_udp_header(this->_header);
          int msg_type = this->_header.msg_type;

          if((msg_type == SIMULATOR_MESSAGE_DATA_CAMERA_GIMBALS)){
               printf("[INFO] SwansonV2::receiveUdpMessage() ---- Received Camera Gimbals Message!\r\n");
               this->readGimbalAngles(p);
               float a1 = (float) this->angles.front_angle / 1000000;
               float a2 = (float) this->angles.right_angle / 1000000;
               float a3 = (float) this->angles.left_angle / 1000000;
               a1 = a1 - 90.0; a2 = a2 - 90.0; a3 = a3 - 90.0;
               msize = sizeof(Sim_Msg_CameraGimbalsCommand) + sizeof(Sim_Msg_Data_Header);
               printf("Received Angle Targets:     %.5f   |    %.5f   |    %.5f \r\n", a1, a2, a3);
          }
          else if(msg_type == SIMULATOR_MESSAGE_DATA_RCCOMMANDS){
               this->readRC(p);
               float normV = (float) this->controls.normalized_speed / 1000000;
               float normOmega = (float) this->controls.normalized_yaw_rate / 1000000;

               float v = normV * _max_speed;
               float omega = normOmega * _max_omega;

               msize = sizeof(Sim_Msg_MotionCommands) + sizeof(Sim_Msg_Data_Header);
               printf("Linear, Angular:     %.5f   |    %.5f \r\n",v, omega);
          }
          else if(msg_type == SIMULATOR_MESSAGE_DATA_MOTION_CONTROL){
               // printf("[INFO] SwansonV2::receiveUdpMessage() ---- Received RC Commands Message!\r\n");
               this->readRC(p);
               float normV = (float) this->controls.normalized_speed / 1000000;
               float normOmega = (float) this->controls.normalized_yaw_rate / 1000000;

               float v = normV * _max_speed;
               float omega = normOmega * _max_omega;
               msize = sizeof(Sim_Msg_MotionCommands) + sizeof(Sim_Msg_Data_Header)-4;
               
               this->readGimbalAngles(p);
               printf("Linear, Angular:     %.5f   |    %.5f |  %d, %d, %d, %d \r\n",v, omega,this->angles.left_angle,this->angles.right_angle,this->angles.front_angle,this->angles.back_angle);
          }
          else if(nBytes < 0){ printf("Negative Bytes Read\r\n"); }

          mtot += msize;
          // printf("[INFO] nBytes = %d, mtot = %d, msize = %d, msg_type = %d\r\n",nBytes,mtot,msize,msg_type);
          // std::cout<<"nBytes, mtot, msg_type: " << nBytes << "    |     " << mtot << "    |     " <<  msg_type << endl;

     } while(mtot < nBytes - 50);

     // printUdpHeader(&_header);
     // this->rc_in->flush();
     _count++;
}

void SwansonV2::read_udp_header(){
	// UDP* udp_line = rc_in;
     // Udp_Msg_Header* data = &udp_header;
     //
	// char* dat = udp_line->read(sizeof(Udp_Msg_Header));
	// memcpy(data, &dat[0],sizeof(Udp_Msg_Header));
     // print_udp_header(data);
}

void SwansonV2::read_udp_commands(){
     UDP* udp_line = rc_in;
     Udp_Msg_Header* head = &udp_header;
     RC_COMMAND_MSG* data = &_controls;

     // printf("Size of RC Msg Data = %d,%d ----- Size of UDP Header = %d",sizeof(data),sizeof(*data),sizeof((*Udp_Msg_Header)));

     char* dat = udp_line->read(sizeof(*data)+20);
     memcpy(head, &dat[0],sizeof(*head)-4);
     memcpy(data, &dat[16],sizeof(*data)+4);

     if(this->flag_verbose == 1){
          cout << "Received UDP Bytes: ";
          for(int i = 0; i < sizeof(*data)+20; i++){
               cout << (int)dat[i] << ", ";
          }
          cout << endl;
     }

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
     vector<float> imu_data = imu->get_all_data();
     vector<float> encoder_data = claws->get_odom_deltas();

     // Reformat into single data vector
     raw_data.push_back(_time);
     raw_data.insert(raw_data.end(), imu_data.begin(), imu_data.end());
     raw_data.insert(raw_data.end(), encoder_data.begin(), encoder_data.end());

     // Miscellaneous Debugging

     return raw_data;
}

void SwansonV2::open_datalog(string file_path){
     datalog.open(file_path + ".csv");
     //time,ax,ay,az,gx,gy,gz,mx,my,mz,roll,pitch,yaw,od,oyaw,ox,oy;
     datalog << "Time (sec), Accel X (m/s^2), Accel Y (m/s^2), Accel Z (m/s^2), Gyro X (rad/s), Gyro Y (rad/s), Gyro Z (rad/s), Mag X (μT), Mag Y (μT), Mag Z (μT), Quat X, Quat Y, Quat Z, Quat W, Roll (deg), Pitch (deg), Yaw (deg), Δdistance (m), ΔYaw (rad), ΔX (m), ΔY (m)\n";
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

void SwansonV2::print_udp_header(Sim_Msg_Data_Header header){
// void SwansonV2::print_udp_header(Udp_Msg_Header* header){

     int32_t header_byte, msg_type, data_type, measurement_type, measurement_length;

     // header_byte = (int32_t) header->id / 1000000;
     // msg_type = (int32_t) header->msg_type / 1000000;
     // data_type = (int32_t) header->data_type / 1000000;
     // measurement_type = (int32_t) header->measurement_type / 1000000;
     // measurement_length = (int32_t) header->measurement_length / 1000000;
     //
	// printf("=========== UDP Packet Info     =================\r\n");
     // printf("UDP Packet Header:\r\n");
     // printf("  Header Byte: %d\r\n", header_byte);
     // printf("  Message Type: %d\r\n", msg_type);
     // printf("  Data Type: %d\r\n", data_type);
     // printf("  Measurement Type: %d\r\n", measurement_type);
     // printf("  Measurement Length: %d\r\n", measurement_length);


     header_byte = (int32_t) header.component_id / 1000000;
     msg_type = (int32_t) header.msg_type / 1000000;
     data_type = (int32_t) header.data_type / 1000000;
     measurement_type = (int32_t) header.measurement_type / 1000000;
     measurement_length = (int32_t) header.measurement_length / 1000000;

	printf("=========== UDP Packet Info     =================\r\n");
     printf("UDP Packet Header:\r\n");
     printf("  Header Byte: %d\r\n", header_byte);
     printf("  Message Type: %d\r\n", msg_type);
     printf("  Data Type: %d\r\n", data_type);
     printf("  Measurement Type: %d\r\n", measurement_type);
     printf("  Measurement Length: %d\r\n", measurement_length);

}
