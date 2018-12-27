#include <unistd.h>
#include <iostream>
#include <vector>
#include <string>
#include <pigpiod_if2.h>

#include "bno055.h"

using namespace std;

BNO055::BNO055(int pi, std::string dev, int baud){
     int handle;
     int err;
     char* device = new char[dev.length()+1];
     std::strcpy(device, dev.c_str());
     if(pi >= 0){
          this->_pi = pi;
          handle = serial_open(pi, device, baud,0);
          if(handle >= 0){
               this->_handle = handle;
               this->_baud = baud;
          }else{
               printf("[ERROR] BNO-055 at %s with Baud rate of %d could not establish UART connection! Exiting...\r\n",device,baud);
               this->_initialized = false;
               std::exit(2);
          }
     }else{
          printf("[ERROR] PigpioD Not properly initialized! Exiting...\r\n");
          this->_initialized = false;
          std::exit(1);
     }
}

BNO055::~BNO055(){
     int err = serial_close(this->_pi, this->_handle);
}

int BNO055::_imu_write_byte(BNO055Register reg, uint8_t byte){
     int err = _imu_write_bytes(reg, &byte, 1);
     return err;
}

int BNO055::_imu_write_bytes(BNO055Register reg, uint8_t* bytes, int length){
     int numBytes = (sizeof(bytes)/sizeof(*bytes));
     printf("[BNO055::_write_bytes] ---- nBytes: %d\r\n", numBytes);

     uint8_t tmp[length+4];
     tmp[0] = 0xAA;
     tmp[1] = 0;
     tmp[2] = (uint8_t)reg;
     tmp[3] = (unsigned)length;
     for(int i = 0; i < length; ++i){
		tmp[i+4] = *((uint8_t*)bytes+i);
	}

     int err = _write(tmp, length);
	if(err >= 0)
		return 0;
	else{
          printf("[BNO055::_write_bytes] ---- ERROR! Unsuccessful write...\r\n");
          return err;
     }

}

int BNO055::_write(uint8_t* bytes, int length, bool ack, int max_trys){
     // # Send a serial command and automatically handle if it needs to be resent
     // # because of a bus error.  If ack is True then an ackowledgement is
     // # expected and only up to the maximum specified attempts will be made
     // # to get a good acknowledgement (default is 5).  If ack is False then
     // # no acknowledgement is expected (like when resetting the device).
     int trys = 0;
     while(trys < max_trys + 1){
          uint8_t recv_data[2] = {0,0};
          // # Flush any pending received data to get into a clean state.
          this->flush();
          // # Send the data.
          int err = serial_write(_pi,_handle,(char*)bytes, length);

          // # Stop if no acknowledgment is expected.
          if(!ack)
               return err;

          // # Read acknowledgement response (2 bytes).
          if(_read(&recv_data[0], 2) == 2){
               // # Stop if there's no bus error (0xEE07 response) and return response bytes.
               // if(!(recv_data[0]==0xEE && recv_data[1]==0x07) )
               if(!(recv_data[0]==0xEE && recv_data[1]==0x01) )
                    return 0;
          }

          // # Else there was a bus error so resend, as recommended in UART app
          trys += 1;
     }

     printf("[BNO055::_write] ---- Exceeded maximum attempts to acknowledge serial command without bus error!\r\n");

     return -4;
}

int BNO055::_read(uint8_t* buf, int length){
     int bytesRead = serial_read(_pi,_handle,(char*)buf,length);

	if(bytesRead > 0){
          printf("[BNO055::_read] ---- # of bytes received: %d\r\n", bytesRead);
     }else
		cout << "Pigpiod: Serial read error" << endl;

	return bytesRead;
}

int BNO055::read(BNO055Register reg, uint8_t *data, int length){
     uint8_t* buf;
     char tmp[4];
     tmp[0] = 0xAA;
     tmp[1] = 1;
     tmp[2] = (uint8_t)reg;
     tmp[3] = (unsigned)length;

     this->flush();
     int err = serial_write(_pi,_handle, tmp, length+2);
     // If bytes successful sent out and ackowledgement received then read in data
	if(err > 0){
          int nRead = _read(buf,length+2);
          if(buf[0]==0xBB && buf[1]==length){
     		memcpy(data,&buf[2],length);
               return 0;
          }
     }else{
          printf("[BNO055::_write_bytes] ---- ERROR! Unsuccessful write...\r\n");
          return err;
     }
}

int BNO055::available(){
	return serial_data_available(_pi,_handle);
}

void BNO055::flush(){
	char* tmp;
	int buf = available();
	serial_read(_pi,_handle,tmp, buf);
}

int BNO055::begin(BNO055OpMode mode){
     int err;
     this->flush();

     if(_imu_write_byte(PAGE0_OPR_MODE,OP_MODE_CONFIG) < 0)
          return -1;
     // if(_imu_write_byte(PAGE0_PWR_MODE,PWR_MODE_NORMAL) < 0)
     //      return -2;
     // if(_imu_write_byte(PAGE_ID,0) < 0)
     //      return -3;
     // if(_imu_write_byte(PAGE0_SYS_TRIGGER,0x00) < 0)
     //      return -4;
     // if(_imu_write_byte(PAGE0_UNIT_SEL,0x83) < 0)
     //      return -5;
     // if(_imu_write_byte(PAGE0_AXIS_MAP_CONFIG,0x24) < 0)
     //      return -6;
     // if(_imu_write_byte(PAGE0_AXIS_MAP_SIGN,0x06) < 0)
     //      return -7;
     // if(_imu_write_byte(PAGE0_OPR_MODE,OP_MODE_NDOF) < 0)
     //      return -8;
}
