#include <iostream>
#include <unistd.h>
#include <string.h>

#include "communication/uart.h"

using namespace std;

int writer(UartDev* device, char* bytes, int num_bytes){
	int size = sizeof(bytes)/sizeof(*bytes);
	int max_trys = 4;
	int trys = 0;
	int bytes_written;
	char _buf[4096];

	device->flush();
	while(trys <= max_trys + 1){
		bytes_written = device->write_bytes(&bytes[0], num_bytes);
		usleep(0.003 * 1000000);
		// printf("UartDev::write_bytes ---- num_bytes , size, bytes_written: %d, %d, %d\r\n",num_bytes,size,bytes_written);
		int nBytes = device->bytes_available();
		char resp[nBytes];
		// this->read_bytes(&_buf[0], nBytes);
		device->read_bytes(&resp[0], nBytes);
		uint8_t resp_header = (resp[0]) & 0xFF;
          uint8_t resp_status = (resp[1]) & 0xFF;
		printf("[INFO] Response Received (header, status): %d,\t%d\r\n", (int)resp_header,(int)resp_status);
		bool resp_check = ((resp_header == 0xEE) && (resp_status == 0x01) );
		if(resp_check){
               break;
          }
		trys += 1;
	}

	return bytes_written;
}

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

     err = writer(serialDev, (char*)bufArr2,5);
     usleep(0.03 * 1000000);
     printf(" ------------------------ \r\n");

     err = writer(serialDev, (char*)bufArr3,5);
     usleep(0.03 * 1000000);
     printf(" ------------------------ \r\n");

     err = writer(serialDev, (char*)bufArr4,4);
     usleep(0.03 * 1000000);
     nBytes = serialDev->bytes_available();
     err = serialDev->read_bytes(&buffer[0], nBytes);
     printf(" ------------------------ \r\n");

     return 0;
}



/** TO COMPILE:

     g++ -w test_udp.cpp -o TestUDP -lpigpiod_if2 -Wall -pthread -std=c++11

*/
