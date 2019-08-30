#include <iostream>
#include <unistd.h>
#include <string.h>

#include "communication/udp.h"

using namespace std;

typedef struct CommunicationHeaderByte{
     int32_t header;
     int32_t msg_type;
     int32_t data_type;
     int32_t measurement_type;
     int32_t measurement_length;
}CommunicationHeaderByte;

int main(int argc, char *argv[]){
     int err, c;
     int port = 35000;
     char* add = "192.168.1.135";
     UDP* rc_in = new UDP(port, add);

     CommunicationHeaderByte* headin;

     sleep(1);
     while(1){
          usleep(0.1 * 1000000);

          // readImu(rc_in, headin, imu_data);
          // readGps(rc_in, headin, gps_data);
          // readLidar(rc_in, headin, lidar_data);
     }

     return 0;
}



/** TO COMPILE:

     g++ -w test_udp.cpp -o TestUDP -lpigpiod_if2 -Wall -pthread -std=c++11

*/
