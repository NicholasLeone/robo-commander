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
#include "utils/utils.h"

using namespace std;

static void readControls(UDP* udp_line, RC_COMMAND_MSG* data){
     int i = 0;

     char* dat = udp_line->read(sizeof(data)+20);
     memcpy(data, &dat[16],sizeof(data)+4);

}

static void readImu(UDP* udp_line, CommunicationHeaderByte* header, Sim_Msg_IMUData* data){

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
     data = (Sim_Msg_IMUData*)&dat[20];

     printImu(*data);
}

static void readGps(UDP* udp_line, CommunicationHeaderByte* header, Sim_Msg_GPSData* data){

     CommunicationHeaderByte* tmpHead;
     char* dat;
     int ready = 0;

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

     header = (CommunicationHeaderByte*)&dat[0];
     data = (Sim_Msg_GPSData*)&dat[20];

     printGps(*data);
}

static void readLidar(UDP* udp_line, CommunicationHeaderByte* header, Sim_Msg_LidarData* data){

     CommunicationHeaderByte* tmpHead;
     char* dat;
     int ready = 0;

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
     header = (CommunicationHeaderByte*)&dat[0];
     data = (Sim_Msg_LidarData*)&dat[20];

     printLidar(*data);
}

int main(int argc, char *argv[]){
     int err, c;
     int port = 35000;
     char* add = "192.168.1.135";
     UDP* rc_in = new UDP(port);

     Sim_Msg_IMUData* imu_data;
     Sim_Msg_GPSData* gps_data;
     Sim_Msg_LidarData* lidar_data;
     CommunicationHeaderByte* headin;

     sleep(1);
     while(1){
          usleep(0.1 * 1000000);

          // readImu(rc_in, headin, imu_data);
          // readGps(rc_in, headin, gps_data);
          readLidar(rc_in, headin, lidar_data);
     }

     return 0;
}



/** TO COMPILE:

     g++ -w test_udp.cpp -o TestUDP -lpigpiod_if2 -Wall -pthread -std=c++11

*/
