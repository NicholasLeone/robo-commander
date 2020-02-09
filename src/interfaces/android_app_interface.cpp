#include <string.h> 	    	// For memcpy
#include <chrono>			// For high_resolution_clock
#include <iostream>			// For printf

#include "base/definitions.h"
#include "interfaces/android_app_interface.h"

using namespace std;
using namespace chrono;

AndroidAppInterface::AndroidAppInterface(){
	/** Variable Initializations */
	this->_count = 0;
	this->_header_count = 0;
     this->_rc_msg_count = 0;
	this->_gimbal_msg_count = 0;

	/** Object Initializations */
     this->_port = 14500;
	this->mUdp = new UDP(this->_port,NULL);

	/** Timing */
	this->_prev_time = high_resolution_clock::now();
	this->_cur_time = high_resolution_clock::now();

	/** Flags */
     this->flag_verbose = false;
     this->debug_timing = false;
}

AndroidAppInterface::AndroidAppInterface(int listenPort, bool verbose){
	/** Variable Initializations */
	this->_count = 0;
	this->_header_count = 0;
     this->_rc_msg_count = 0;
	this->_gimbal_msg_count = 0;

	/** Object Initializations */
     this->_port = listenPort;
	this->mUdp = new UDP(listenPort,NULL);

	/** Timing */
	this->_prev_time = high_resolution_clock::now();
	this->_cur_time = high_resolution_clock::now();

	/** Flags */
     this->flag_verbose = verbose;
	this->debug_timing = false;
}

AndroidAppInterface::~AndroidAppInterface(){
     printf("AndroidAppInterface Shutting Down...\r\n");
     delete this->mUdp;
}

void AndroidAppInterface::readHeader(char* msgBuffer){
     Sim_Msg_Data_Header* data = &header;
     memcpy(data, &msgBuffer[0],sizeof(Sim_Msg_Data_Header));
     this->_header_count++;
}

void AndroidAppInterface::readRC(char* msgBuffer){
     Sim_Msg_MotionCommands* data = &controls;
	memcpy(data, &msgBuffer[16],sizeof(*data));
	this->_rc_msg_count++;
}

void AndroidAppInterface::readGimbalAngles(char* msgBuffer){
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

int AndroidAppInterface::receiveUdpMessage(bool verbose, bool debug){
	int ret = 0;
	int mtot = 0;
	int msize = 0;
     char* p = nullptr;
	char* msgBuf = nullptr;

	if(this->debug_timing){
		this->_cur_time = high_resolution_clock::now();
		duration<float> time_span = duration_cast<duration<float>>(this->_cur_time - this->_prev_time);
		float dt = time_span.count();
		printf("Elapsed time = %.6f seconds\r\n",dt);
	}

     msgBuf = this->mUdp->read(4096,true,15);
     int nBytes = this->mUdp->bytesRead;
	int timeO = this->mUdp->timeout_wait;
     // if(timeO) printf("Received %d bytes over UDP. Timeout Wait = %d\r\n",nBytes,timeO);
     do{
          p = &msgBuf[mtot];
          this->readHeader(p);
          int msg_type = this->header.msg_type;

          if((msg_type == SIMULATOR_MESSAGE_DATA_CAMERA_GIMBALS)){
               if(debug) printf("[INFO] AndroidAppInterface::receiveUdpMessage() ---- Received Camera Gimbals Message!\r\n");
               this->readGimbalAngles(p);
			msize = sizeof(Sim_CameraGimbalsCommand) + sizeof(Sim_Msg_Data_Header);
               float a1 = (float) this->angles.front_angle / 1000000;
               float a2 = (float) this->angles.right_angle / 1000000;
               float a3 = (float) this->angles.left_angle / 1000000;
               a1 = a1 - 90.0; a2 = a2 - 90.0; a3 = a3 - 90.0;
			if(verbose) printf("Received Angle Targets:     %.5f   |    %.5f   |    %.5f \r\n", a1, a2, a3);
			ret = 2;
		}
          else if(msg_type == SIMULATOR_MESSAGE_DATA_RCCOMMANDS){
			if(debug) printf("[INFO] AndroidAppInterface::receiveUdpMessage() ---- Received RC Commands Message!\r\n");
               this->readRC(p);
			msize = sizeof(Sim_Msg_MotionCommands) + sizeof(Sim_Msg_Data_Header);

			float normV = (float) this->controls.normalized_speed / 1000000;
               float normW = (float) this->controls.normalized_yaw_rate / 1000000;
			this->mData.normalized_speed = normV;
			this->mData.normalized_turn_rate = normW;
			if(verbose) printf("Linear, Angular:     %.5f   |    %.5f \r\n",normV, normW);
			ret = 3;
		}
          else if(msg_type == SIMULATOR_MESSAGE_DATA_MOTION_CONTROL){
			if(debug) printf("[INFO] AndroidAppInterface::receiveUdpMessage() ---- Received Message with RC Commands and Camera Gimbal Angles!\r\n");
               this->readRC(p);
			this->readGimbalAngles(p);
			msize = sizeof(Sim_Msg_MotionCommands) + sizeof(Sim_Msg_Data_Header)-4;

               float normV = (float) this->controls.normalized_speed / 1000000;
               float normW = (float) this->controls.normalized_yaw_rate / 1000000;
			this->mData.normalized_speed = normV;
			this->mData.normalized_turn_rate = normW;
			this->mData.front_cam_angle = this->angles.front_angle;
			this->mData.left_cam_angle = this->angles.left_angle;
			this->mData.right_cam_angle = this->angles.right_angle;
			this->mData.back_cam_angle = this->angles.back_angle;
			if(verbose) printf("Linear, Angular:     %.5f   |    %.5f |  %d, %d, %d, %d \r\n", normV, normW,this->angles.left_angle,this->angles.right_angle, this->angles.front_angle,this->angles.back_angle);
			ret = 1;
		}
		else if(nBytes < 0){  // Timeout occurred
			// printf("UDP Timeout Occurred\r\n");
			ret = -1;
		}
		mtot += msize;
		if(debug) printf("[INFO] nBytes = %d, mtot = %d, msize = %d, msg_type = %d\r\n",nBytes,mtot,msize,msg_type);
     } while(mtot < nBytes - 50);

	if(this->debug_timing) this->_prev_time = this->_cur_time;
     this->_count++;
	return ret;
}

AndroidInterfaceData AndroidAppInterface::getReceivedData(){
	return this->mData;
}

void AndroidAppInterface::print_udp_header(Sim_Msg_Data_Header header){
     int32_t header_byte, msg_type, data_type, measurement_type, measurement_length;

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

void AndroidAppInterface::read_udp_header(){
	// UDP* udp_line = mUdp;
	// Udp_Msg_Header* data = &udp_header;
	//
	// char* dat = udp_line->read(sizeof(Udp_Msg_Header));
	// memcpy(data, &dat[0],sizeof(Udp_Msg_Header));
	// print_udp_header(data);
}

void AndroidAppInterface::read_udp_commands(){
	// UDP* udp_line = mUdp;
	// Udp_Msg_Header* head = &udp_header;
	// RC_COMMAND_MSG* data = &_controls;
	//
	// // printf("Size of RC Msg Data = %d,%d ----- Size of UDP Header = %d",sizeof(data),sizeof(*data),sizeof((*Udp_Msg_Header)));
	//
	// char* dat = udp_line->read(sizeof(*data)+20);
	// memcpy(head, &dat[0],sizeof(*head)-4);
	// memcpy(data, &dat[16],sizeof(*data)+4);
	//
	// if(this->flag_verbose == 1){
	// 	cout << "Received UDP Bytes: ";
	// 	for(int i = 0; i < sizeof(*data)+20; i++){
	// 		cout << (int)dat[i] << ", ";
	// 	}
	// 	cout << endl;
	// }
}
