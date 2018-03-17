#include <fstream>
#include <math.h>
#include <signal.h>
#include <string.h>
#include <algorithm>

#include "utils.h"

// #define GTSAM_LIBRARY_INCLUDED

using namespace std;

float convertRadians2Degrees(float angle){
     float degr = angle * 180.0 / M_PI;
     // cout << "Converted Angle (deg): " << degr << endl;
     return degr;
}

float convertDegrees2Radians(float angle){

     float rad = angle * M_PI / 180.0;
     // cout << "Converted Angle (rad): " << rad << endl;
     return rad;
}


int convertSpdRatio2Pulse(float spd_ratio, int max, int min, int neutral){

     float dPulse = (max - min)/2;
     float pulse = (float) dPulse * spd_ratio + (float) neutral;

     // cout << "Converted Pulse: " << pulse << endl;
     return (int) pulse;
}

void LoadInitialVariables(const string &fileName, map<string, float> &variables){
    variables.clear();

    char name[256];
    float value;
    int numLines;
    string line,tmpline;

	ifstream myfile(fileName.c_str());

	numLines = 0;

	while(getline(myfile, tmpline)){
		if (sscanf(tmpline.c_str(), "%s = %f", name, &value) == 2){
            variables[name] = value;
		}
		++numLines;
	}
}

void LoadStringVariables(const string &fileName, map<string, string> &variables){
    variables.clear();

    char name[256];
    char value[1028];
    int numLines;
    string line,tmpline;

	ifstream myfile(fileName.c_str());

	numLines = 0;

	while(getline(myfile, tmpline)){
		if (sscanf(tmpline.c_str(), "%s = %s", name, &value) == 2){
            variables[name] = value;
		}
		++numLines;
	}
}

int countData(string s, char delimiter){
     size_t num = count(s.begin(), s.end(), delimiter);
     num = num + 1;
     return num;
}

int countLines(const string &fileName){

    int numLines;
    string tmpline;

	ifstream myfile(fileName.c_str());

	numLines = 0;

	while(getline(myfile, tmpline)){
		++numLines;
	}

	return numLines;
}


int extract_bit(int inputByte, int bitLocation){

     int tmpBit = (inputByte >> bitLocation) & ~(~0 << 1);
     return tmpBit;

}

int extract_bits(int inputByte, int msb, int lsb){

     int numBits = msb - lsb + 1;
     int tmpBits = (inputByte >> lsb) & ~(~0 << numBits);

     return tmpBits;
}

float unpackFloat(char* buffer, int *i){
    float out;
    int b[4];

    *i += 4;

     for(int j = 0;j < 4; j++){
		//cout << (int)buffer[j];
          b[j] = (int) buffer[j];
     }

     int16_t B1 = b[0] << 8 | (b[1] & 0xFF);
     int16_t B2 = b[2] << 8 | (b[3] & 0xFF);
     int32_t tmp = B1 << 16 | (B2 & 0xFFFF);

     memcpy(&out, &tmp, sizeof(tmp));
     return out;
}

vector<float> parseFloat(string s, string delimiter){
     size_t pos = 0;
     string token;
     int i = 0;
     int tmpLength;
     int numVar = countData(s, ',');
     vector<float> tmpData(numVar);

     while ((pos = s.find(delimiter)) != string::npos) {
          token = s.substr(0, pos);
          tmpLength = token.size();
          char tmpChar[tmpLength];
          token.copy(tmpChar, tmpLength);

          tmpData.at(i) = strtof(tmpChar, NULL);

          i++;
          s.erase(0, pos + delimiter.length());
     }

     tmpLength = s.size();
     char tmpChar[tmpLength];
     s.copy(tmpChar, tmpLength);

     tmpData.at(i) = strtof(tmpChar, NULL);

     return tmpData;

}

void attach_CtrlC(void_int_fun func2call){
     struct sigaction sigIntHandler;
     sigIntHandler.sa_handler = func2call;
     sigemptyset(&sigIntHandler.sa_mask);
     sigIntHandler.sa_flags = 0;
     sigaction(SIGINT, &sigIntHandler, NULL);

}

void attach_CtrlZ(void_int_fun func2call){

     struct sigaction sigUpHandler;
     sigUpHandler.sa_handler = func2call;
     sigemptyset(&sigUpHandler.sa_mask);
     sigUpHandler.sa_flags = 0;
     sigaction(SIGTSTP, &sigUpHandler, NULL);
}


#ifdef GTSAM_LIBRARY_INCLUDED
#define foreach BOOST_FOREACH

void writeResults(Values &results, string outputFile){
     ofstream resultFile(outputFile.c_str());

     Values::ConstFiltered<Pose2> result_poses = results.filter<Pose2>();
     foreach (const Values::ConstFiltered<Pose2>::KeyValuePair& key_value, result_poses){
          Pose2 p = key_value.value;
          string k = Symbol(key_value.key);
          resultFile << k << " " << p.x() << " " << p.y() << " " << p.theta() << endl;
     }
}
#endif

// void printImu(Sim_Msg_IMUData data){
//
//      float ax, ay, az, gx, gy, gz;
//      float cov_ax, cov_ay, cov_az, cov_gx, cov_gy, cov_gz;
//      float bias_ax, bias_ay, bias_az, bias_gx, bias_gy, bias_gz;
//      float ox, oy, oz, ow;
//      float cov_ox, cov_oy, cov_oz;
//      float bias_ox, bias_oy, bias_oz, bias_ow;
//
//      ax = (float) data.accel.x / 1000000; ay = (float) data.accel.y / 1000000; az = (float) data.accel.z / 1000000;
//      gx = (float) data.gyro.x / 1000000; gy = (float) data.gyro.y / 1000000; gz = (float) data.gyro.z / 1000000;
//
//      cov_ax = (float) data.accel.covariance.x / 1000000; cov_ay = (float) data.accel.covariance.y / 1000000; cov_az = (float) data.accel.covariance.z / 1000000;
//      cov_gx = (float) data.gyro.covariance.x / 1000000; cov_gy = (float) data.gyro.covariance.y / 1000000; cov_gz = (float) data.gyro.covariance.z / 1000000;
//
//      bias_ax = (float) data.accel.bias.x / 1000000; bias_ay = (float) data.accel.bias.y / 1000000; bias_az = (float) data.accel.bias.z / 1000000;
//      bias_gx = (float) data.gyro.bias.x / 1000000; bias_gy = (float) data.gyro.bias.y / 1000000; bias_gz = (float) data.gyro.bias.z / 1000000;
//
//      ox = (float) data.orientation.x / 1000000; oy = (float) data.orientation.y / 1000000; oz = (float) data.orientation.z / 1000000; ow = (float) data.orientation.w / 1000000;
//      cov_ox = (float) data.orientation.covariance.roll / 1000000; cov_oy = (float) data.orientation.covariance.pitch / 1000000; cov_oz = (float) data.orientation.covariance.yaw / 1000000;
//      bias_ox = (float) data.orientation.bias.x / 1000000; bias_oy = (float) data.orientation.bias.y / 1000000; bias_oz = (float) data.orientation.bias.z / 1000000; bias_ow = (float) data.orientation.bias.w / 1000000;
//
//      printf("=========== IMU Measurement     =================\r\n");
//      printf("Accelerometer Data:\r\n");
//      printf("  Accelerations (X, Y, Z): %.5f,   %.5f,     %.5f\r\n", ax,ay,az);
//      printf("  Covariances (X, Y, Z): %.5e,   %.5e,     %.5e\r\n", cov_ax,cov_ay,cov_az);
//      printf("  Biases (X, Y, Z): %.5f,   %.5f,     %.5f\r\n", bias_ax,bias_ay,bias_az);
//      printf("Gyro Data:\r\n");
//      printf("  Angular Velocities (X, Y, Z): %.5f,   %.5f,     %.5f\r\n", gx,gy,gz);
//      printf("  Covariances (X, Y, Z): %.5e,   %.5e,     %.5e\r\n", cov_gx,cov_gy,cov_gz);
//      printf("  Biases (X, Y, Z): %.5f,   %.5f,     %.5f\r\n", bias_gx,bias_gy,bias_gz);
//      printf("Orientation Data:\r\n");
//      printf("  Quaternions (X, Y, Z, W): %.5f,   %.5f,     %.5f, %.5f\r\n", ox,oy,oz,ow);
//      printf("  Covariances (Roll, Pitch, Yaw): %.5e,   %.5e,     %.5e\r\n", cov_ox,cov_oy,cov_oz);
//      printf("  Biases (X, Y, Z, W): %.5f,   %.5f,     %.5f, %.5f\r\n", bias_ox,bias_oy,bias_oz,bias_ow);
// }
//
// void printGps(Sim_Msg_GPSData data){
//
//      float t, lat, lon, alt;
//      float vx, vy, vz, cov_x, cov_y, cov_z;
//      uint8_t cov_type;
//      uint16_t srv_type;
//      int8_t status;
//
//      t = (float) data.time / 1000000;
//      cov_type = data.covariance_type; srv_type = data.service; status = data.status;
//
//      lat = (double) data.latitude / 1000000; lon = (double) data.longitude / 1000000; alt = (double) data.altitude / 1000000;
//      vx = (float) data.velocity.x / 1000000; vy = (float) data.velocity.y / 1000000; vz = (float) data.velocity.z / 1000000;
//      cov_x = (float) data.covariance.x / 1000000; cov_y = (float) data.covariance.y / 1000000; cov_z = (float) data.covariance.z / 1000000;
//
//      printf("=========== GPS Measurement     =================\r\n");
//      printf("GPS Info Data:\r\n");
//      printf("  Fix Status: %d\r\n", status);
//      printf("  GPS Service Type: %d\r\n", srv_type);
//      printf("  Covariance Type: %d\r\n", cov_type);
//      printf("GPS Data:\r\n");
//      printf("  Timestamp: %.4f\r\n", t);
//      printf("  Coordinates (Lat, Long, Alt): %.9f,   %.9f,     %.9f\r\n", lat,lon,alt);
//      printf("  Velocity (X, Y, Z): %.5e,   %.5e,     %.5e\r\n", vx, vy, vz);
//      printf("  Covariances (Lat, Long, Alt): %.5e,   %.5e,     %.5e\r\n", cov_x, cov_y, cov_z);
// }
//
// void printLidar(Sim_Msg_LidarData data){
//
//      float angle_min = (float) data.angle_min / 1000000;
//      float angle_max = (float) data.angle_max / 1000000;
//      float dAngle = (float) data.dAngle / 1000000;
//      float scan_time = (float) data.scan_time / 1000000;
//      float dTime = (float) data.dTime / 1000000;
//      float range_min = (float) data.range_min / 1000000;
//      float range_max = (float) data.range_max / 1000000;
//      // float ranges[];
//      // float intensities[];
//
//      printf("=========== Lidar Measurement     =================\r\n");
//      printf("LiDAR Info Data:\r\n");
//      printf("  Angle Limits: %.4f  |    %.4f\r\n", angle_min,angle_max);
//      printf("  Range Limits: %.4f  |    %.4f\r\n", range_min,range_max);
//      printf("  Δangle, Δtime: %.4f  |    %.6f\r\n", dAngle,dTime);
//      printf("LiDAR Data:\r\n");
//      printf("  Ranges: ");
// 	cout << "\r\nSize: " << sizeof(data.ranges);
// 	for(int i=0;i<sizeof(data.ranges);i++){
// 		// cout << (float) data->ranges[i] / 1000000 << ", ";
// 	}
//      printf("  Intensities: \r\n");
// }

// void printUdpHeader(CommunicationHeaderByte* header){
//
//      int header_byte, msg_type, data_type, measurement_type, measurement_length;
//
//      header_byte = header->header;
//      msg_type = header->msg_type;
//      data_type = header->data_type;
//      measurement_type = header->measurement_type;
//      measurement_length = header->measurement_length;
//
// 	printf("=========== UDP Packet Info     =================\r\n");
//      printf("UDP Packet Header:\r\n");
//      printf("  Header Byte: %d\r\n", header_byte);
//      printf("  Message Type: %d\r\n", msg_type);
//      printf("  Data Type: %d\r\n", data_type);
//      printf("  Measurement Type: %d\r\n", measurement_type);
//      printf("  Measurement Length: %d\r\n", measurement_length);
//
// }


// int sendUdp(int _port, char* _add, CommunicationHeaderByte* header, Sim_Msg_IMUData data){
//
//      char _buf[4096];
//      memset(_buf, 0, sizeof(_buf));
//
//      memcpy(&_buf[0], &header->header, sizeof(int));
//      memcpy(&_buf[4], &header->msg_type, sizeof(int));
//      memcpy(&_buf[8], &header->data_type, sizeof(int));
//      memcpy(&_buf[12], &header->measurement_type, sizeof(int));
//      memcpy(&_buf[16], &header->measurement_length, sizeof(int));
//      memcpy(&_buf[20], &data, sizeof(data));
//
//      int err = udp_sock->write(_buf,sizeof(*header)+sizeof(data),_add,_port);
//
//      return err;
//
// }
//
// int sendUdp(int _port, char* _add, CommunicationHeaderByte* header, Sim_Msg_GPSData data){
//
//      char _buf[4096];
//      memset(_buf, 0, sizeof(_buf));
//
//      memcpy(&_buf[0], &header->header, sizeof(int));
//      memcpy(&_buf[4], &header->msg_type, sizeof(int));
//      memcpy(&_buf[8], &header->data_type, sizeof(int));
//      memcpy(&_buf[12], &header->measurement_type, sizeof(int));
//      memcpy(&_buf[16], &header->measurement_length, sizeof(int));
//      memcpy(&_buf[20], &data, sizeof(data));
//
//      int err = udp_sock->write(_buf,sizeof(*header)+sizeof(data),_add,_port);
//
//      return err;
//
// }
//
// int sendUdp(int _port, char* _add, CommunicationHeaderByte* header, Sim_Msg_LidarData data){
//
// 	// int num_bytes = sizeof(*header) + sizeof(data) - sizeof(data.ranges) + sizeof(data.ranges)*sizeof(int32_t);
//
//      char _buf[10000];
//      memset(_buf, 0, sizeof(_buf));
//
//      memcpy(&_buf[0], &header->header, sizeof(int));
//      memcpy(&_buf[4], &header->msg_type, sizeof(int));
//      memcpy(&_buf[8], &header->data_type, sizeof(int));
//      memcpy(&_buf[12], &header->measurement_type, sizeof(int));
//      memcpy(&_buf[16], &header->measurement_length, sizeof(int));
//      memcpy(&_buf[20], &data, sizeof(data));
//
//      int err = udp_sock->write(_buf,sizeof(*header)+sizeof(data),_add,_port);
//
//      return err;
//
// }
