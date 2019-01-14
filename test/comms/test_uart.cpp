#include <iostream>
#include <unistd.h>
#include <string.h>

#include "uart.h"

using namespace std;

int main(int argc, char *argv[]){
     int err, c, nBytes;
     char* add = "/dev/ttyUSB0";
     char buffer[4096];

     UartDev* serialDev = new UartDev(add, 115200);

     uint8_t bufArr1[5] = {0xAA, 0x00, 0x07, 0x01, 0x00};
     uint8_t bufArr2[5] = {0xAA, 0x00, 0x3D, 0x01, 0x00};
     uint8_t bufArr3[5] = {0xAA, 0x00, 0x3D, 0x01, 0x00};
     uint8_t bufArr4[4] = {0xAA, 0x01, 0x00, 0x01};

     serialDev->bytes_available();
     serialDev->flush();

     // Start
     err = serialDev->write_bytes((char*)bufArr1,5);
     usleep(0.03 * 1000000);
     printf(" ------------------------ \r\n");

     err = serialDev->writer((char*)bufArr2,5);
     usleep(0.03 * 1000000);
     printf(" ------------------------ \r\n");

     err = serialDev->writer((char*)bufArr3,5);
     usleep(0.03 * 1000000);
     printf(" ------------------------ \r\n");

     err = serialDev->writer((char*)bufArr4,4);
     usleep(0.03 * 1000000);
     nBytes = serialDev->bytes_available();
     err = serialDev->read_bytes(&buffer[0], nBytes);
     printf(" ------------------------ \r\n");

     return 0;
}



/** TO COMPILE:

     g++ -w test_udp.cpp -o TestUDP -lpigpiod_if2 -Wall -pthread -std=c++11

*/
