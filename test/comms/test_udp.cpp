#include <stdio.h>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <iostream>

#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>

#include "udp.h"

using namespace std;
static const int TEST_UDP_WRITE = 0;

static void readControls(UDP* udp_line, RC_COMMAND_MSG* data){
     int i = 0;

     char* dat = udp_line->read(sizeof(data)+20);
     memcpy(data, &dat[16],sizeof(data)+4);

}

static void printHeader(CommunicationHeaderByte* header){

     int header_byte, msg_type, data_type, measurement_type, measurement_length;

     header_byte = header->header;
     msg_type = header->msg_type;
     data_type = header->data_type;
     measurement_type = header->measurement_type;
     measurement_length = header->measurement_length;

     printf("================================\r\n");
     printf("UDP Packet Header:\r\n");
     printf("  Header Byte: %d\r\n", header_byte);
     printf("  Message Type: %d\r\n", msg_type);
     printf("  Data Type: %d\r\n", data_type);
     printf("  Measurement Type: %d\r\n", measurement_type);
     printf("  Measurement Length: %d\r\n", measurement_length);

}

static void printGps(CommunicationHeaderByte* header, Sim_GPSData* data){

     int header_byte, msg_type, data_type, measurement_type, measurement_length;
     float t, lat, lon, alt;
     float vx, vy, vz, cov_x, cov_y, cov_z;
     uint8_t cov_type;
     uint16_t srv_type;
     int8_t status;

     header_byte = header->header;
     msg_type = header->msg_type;
     data_type = header->data_type;
     measurement_type = header->measurement_type;
     measurement_length = header->measurement_length;

     t = data->time;
     cov_type = data->covariance_type; srv_type = data->service; status = data->status;

     lat = data->latitude; lon = data->longitude; alt = data->altitude;
     vx = data->velocity.x; vy = data->velocity.y; vz = data->velocity.z;
     cov_x = data->covariance.x; cov_y = data->covariance.y; cov_z = data->covariance.z;

     printf("=========== GPS Measurement     =================\r\n");
     printf("UDP Packet Header:\r\n");
     printf("  Header Byte: %d\r\n", header_byte);
     printf("  Message Type: %d\r\n", msg_type);
     printf("  Data Type: %d\r\n", data_type);
     printf("  Measurement Type: %d\r\n", measurement_type);
     printf("  Measurement Length: %d\r\n", measurement_length);
     printf("GPS Info Data:\r\n");
     printf("  Fix Status: %d\r\n", status);
     printf("  GPS Service Type: %d\r\n", srv_type);
     printf("  Covariance Type: %d\r\n", cov_type);
     printf("GPS Data:\r\n");
     printf("  Timestamp: %.4f\r\n", t);
     printf("  Coordinates (Lat, Long, Alt): %.5f,   %.5f,     %.5f\r\n", lat,lon,alt);
     printf("  Velocity (X, Y, Z): %.5f,   %.5f,     %.5f\r\n", vx, vy, vz);
     printf("  Covariances (Lat, Long, Alt): %.5f,   %.5f,     %.5f\r\n", cov_x, cov_y, cov_z);
}


static void printLidar(CommunicationHeaderByte* header, Sim_LidarData* data){

     int header_byte, msg_type, data_type, measurement_type, measurement_length;

     header_byte = header->header;
     msg_type = header->msg_type;
     data_type = header->data_type;
     measurement_type = header->measurement_type;
     measurement_length = header->measurement_length;

     float angle_min = data->angle_min;
     float angle_max = data->angle_max;
     float dAngle = data->dAngle;
     float scan_time = data->scan_time;
     float dTime = data->dTime;
     float range_min = data->range_min;
     float range_max = data->range_max;
     // float ranges[];
     // float intensities[];

     printf("=========== Lidar Measurement     =================\r\n");
     printf("UDP Packet Header:\r\n");
     printf("  Header Byte: %d\r\n", header_byte);
     printf("  Message Type: %d\r\n", msg_type);
     printf("  Data Type: %d\r\n", data_type);
     printf("  Measurement Type: %d\r\n", measurement_type);
     printf("  Measurement Length: %d\r\n", measurement_length);
     printf("LiDAR Info Data:\r\n");
     printf("  Angle Limits: %.4f  |    %.4f\r\n", angle_min,angle_max);
     printf("  Range Limits: %.4f  |    %.4f\r\n", range_min,range_max);
     printf("  Δangle, Δtime: %.4f  |    %.4f\r\n", dAngle,dTime);
     printf("LiDAR Data:\r\n");
     printf("  Ranges: \r\n");
     printf("  Intensities: \r\n");
}



static void printImu(CommunicationHeaderByte* header, Sim_IMUData* data){

     int header_byte, msg_type, data_type, measurement_type, measurement_length;
     float ax, ay, az, gx, gy, gz;
     float cov_ax, cov_ay, cov_az, cov_gx, cov_gy, cov_gz;
     float bias_ax, bias_ay, bias_az, bias_gx, bias_gy, bias_gz;
     float ox, oy, oz, ow;
     float cov_ox, cov_oy, cov_oz;
     float bias_ox, bias_oy, bias_oz, bias_ow;

     header_byte = header->header;
     msg_type = header->msg_type;
     data_type = header->data_type;
     measurement_type = header->measurement_type;
     measurement_length = header->measurement_length;

     ax = data->accel.x; ay = data->accel.y; az = data->accel.z;
     gx = data->gyro.x; gy = data->gyro.y; gz = data->gyro.z;

     cov_ax = data->accel.covariance.x; cov_ay = data->accel.covariance.y; cov_az = data->accel.covariance.z;
     cov_gx = data->gyro.covariance.x; cov_gy = data->gyro.covariance.y; cov_gz = data->gyro.covariance.z;

     bias_ax = data->accel.bias.x; bias_ay = data->accel.bias.y; bias_az = data->accel.bias.z;
     bias_gx = data->gyro.bias.x; bias_gy = data->gyro.bias.y; bias_gz = data->gyro.bias.z;

     ox = data->orientation.x; oy = data->orientation.y; oz = data->orientation.z; ow = data->orientation.w;
     cov_ox = data->orientation.covariance.roll; cov_oy = data->orientation.covariance.pitch; cov_oz = data->orientation.covariance.yaw;
     bias_ox = data->orientation.bias.x; bias_oy = data->orientation.bias.y; bias_oz = data->orientation.bias.z; bias_ow = data->orientation.bias.w;

     printf("================================\r\n");
     printf("UDP Packet Header:\r\n");
     printf("  Header Byte: %d\r\n", header_byte);
     printf("  Message Type: %d\r\n", msg_type);
     printf("  Data Type: %d\r\n", data_type);
     printf("  Measurement Type: %d\r\n", measurement_type);
     printf("  Measurement Length: %d\r\n", measurement_length);
     printf("Accelerometer Data:\r\n");
     printf("  Accelerations (X, Y, Z): %.5f,   %.5f,     %.5f\r\n", ax,ay,az);
     printf("  Covariances (X, Y, Z): %.5f,   %.5f,     %.5f\r\n", cov_ax,cov_ay,cov_az);
     printf("  Biases (X, Y, Z): %.5f,   %.5f,     %.5f\r\n", bias_ax,bias_ay,bias_az);
     printf("Gyro Data:\r\n");
     printf("  Angular Velocities (X, Y, Z): %.5f,   %.5f,     %.5f\r\n", gx,gy,gz);
     printf("  Covariances (X, Y, Z): %.5f,   %.5f,     %.5f\r\n", cov_gx,cov_gy,cov_gz);
     printf("  Biases (X, Y, Z): %.5f,   %.5f,     %.5f\r\n", bias_gx,bias_gy,bias_gz);
     printf("Orientation Data:\r\n");
     printf("  Quaternions (X, Y, Z, W): %.5f,   %.5f,     %.5f, %.5f\r\n", ox,oy,oz,ow);
     printf("  Covariances (Roll, Pitch, Yaw): %.5f,   %.5f,     %.5f\r\n", cov_ox,cov_oy,cov_oz);
     printf("  Biases (X, Y, Z, W): %.5f,   %.5f,     %.5f, %.5f\r\n", bias_ox,bias_oy,bias_oz,bias_ow);

}

static void readImu(UDP* udp_line, CommunicationHeaderByte* header, Sim_IMUData* data){

     CommunicationHeaderByte* tmpHead;
     char* dat;
     int ready = 0;

     //printf("Data Size, &Data Size, *Data Size, Data/Data size: %d, %d, %d, %d\r\n",sizeof(data),sizeof(&data),sizeof(*data),sizeof(data)/sizeof(data[0]));

     while(ready == 0){
          dat = udp_line->read(sizeof(*data)+sizeof(*header));
          tmpHead = (CommunicationHeaderByte*)&dat[0];

          int data_type = tmpHead->data_type;
          // TODO: Add in functionality to check for specific sensor index (needed for multiple same type of sensors)

          if(data_type == SIMULATOR_DATA_IMU)
               ready = 1;
          else
               ready = 0;
     }

     header = (CommunicationHeaderByte*)&dat[0];
     data = (Sim_IMUData*)&dat[20];

     printImu(header, data);
}

static void readGps(UDP* udp_line, CommunicationHeaderByte* header, Sim_GPSData* data){

     CommunicationHeaderByte* tmpHead;
     char* dat;
     int ready = 0;

     // printf("Data Size, &Data Size, *Data Size, Data/Data size: %d, %d, %d, %d\r\n",sizeof(data),sizeof(&data),sizeof(*data),sizeof(data)/sizeof(data[0]));

     while(ready == 0){
          dat = udp_line->read(sizeof(*data)+sizeof(*header));
          tmpHead = (CommunicationHeaderByte*)&dat[0];

          int data_type = tmpHead->data_type;
          // TODO: Add in functionality to check for specific sensor index (needed for multiple same type of sensors)

          if(data_type == SIMULATOR_DATA_GPS)
               ready = 1;
          else
               ready = 0;
     }

     //int dType = header
     header = (CommunicationHeaderByte*)&dat[0];
     data = (Sim_GPSData*)&dat[20];

     //printGps(header, data);
}

static void readLidar(UDP* udp_line, CommunicationHeaderByte* header, Sim_LidarData* data){

     CommunicationHeaderByte* tmpHead;
     char* dat;
     int ready = 0;

     // printf("Data Size, &Data Size, *Data Size, Data/Data size: %d, %d, %d, %d\r\n",sizeof(data),sizeof(&data),sizeof(*data),sizeof(data)/sizeof(data[0]));

     while(ready == 0){
          dat = udp_line->read(sizeof(*data)+sizeof(*header)+4);
          tmpHead = (CommunicationHeaderByte*)&dat[0];

          int data_type = tmpHead->data_type;
          // TODO: Add in functionality to check for specific sensor index (needed for multiple same type of sensors)

          if(data_type == SIMULATOR_DATA_LIDAR)
               ready = 1;
          else
               ready = 0;
     }

     //int dType = header
     header = (CommunicationHeaderByte*)&dat[0];
     data = (Sim_LidarData*)&dat[20];

     printLidar(header, data);
}

static int writeImu(UDP* udp_line, int _port, char* _add, CommunicationHeaderByte* header){

     static Sim_IMUData imu;
     static char buf[10000];
     imu.accel.x = 10;
     imu.accel.y = 11;
     imu.accel.z = 12;

     memcpy(&buf[0], &header->header, sizeof(int));
     memcpy(&buf[4], &header->msg_type, sizeof(int));
     memcpy(&buf[8], &header->data_type, sizeof(int));
     memcpy(&buf[12], &header->measurement_type, sizeof(int));
     memcpy(&buf[16], &header->measurement_length, sizeof(int));
     memcpy(&buf[20], &imu, sizeof(imu));

     char* pre_s = "Hey = ";
     char s[4+sizeof(pre_s)+4];
     sprintf(s,"Hey = %d\r\n",header->header);

	// int err = udp_line->write(s,sizeof(s),_add,_port);
     int err = udp_line->write(buf,sizeof(&header)+sizeof(imu)+4,_add,_port);

     cout << s << endl;

     return err;
}

int main(int argc, char *argv[]){
     int err, c;
     int port = 35000;
     char* add = "192.168.1.135";
     UDP* rc_in = new UDP(port);

     // UDP* rc_in = new UDP(port,NULL);

     RC_COMMAND_MSG controls;
     Sim_IMUData* imu_data;
     Sim_GPSData* gps_data;
     Sim_LidarData* lidar_data;
     CommunicationHeaderByte* headin;
     CommunicationHeaderByte header;

     header.msg_type = SIMULATOR_MESSAGE_SENSOR_DATA;
     header.data_type = SIMULATOR_DATA_IMU;
     header.measurement_type = SIMULATOR_MEASUREMENT_SINGLE;
     header.measurement_length = 1;

     sleep(1);
     if(TEST_UDP_WRITE != 1){
          while(1){
               usleep(0.1 * 1000000);
               // readControls(rc_in,&controls);
               //
               // float accel = controls.speed;
               // float omega = controls.yaw;
               //
               // cout << "Linear Speed, Angular Speed: " << accel << "    |    " << omega << "\r\n";

               // char* dat = rc_in->read(sizeof(sensor_data)+24);
               // sensor_data = (Sim_IMUData*)&dat[20];
               // cout << (float)sensor_data->accel.z << endl;

               // readImu(rc_in, headin, imu_data);
               // readGps(rc_in, headin, gps_data);
               readLidar(rc_in, headin, lidar_data);
          }
     }else{
          int count = 0;
          while(1){
               usleep(1 * 1000000);
               header.header = count;
               count++;
               // cout << "Count: " << count << endl;
               int err = writeImu(rc_in,port, add,&header);
          }

     }
     return 0;
}



/** TO COMPILE:

     g++ -w test_udp.cpp -o TestUDP -lpigpiod_if2 -Wall -pthread -std=c++11

*/
