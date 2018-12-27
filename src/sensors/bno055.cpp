#include <unistd.h>
#include <iostream>
#include <vector>
#include <string>
#include <pigpiod_if2.h>

#include "bno055.h"

using namespace std;

BNO055::BNO055(int pi, std::string dev, int baud){
     /** ----- Open UART -----*/
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

     /** ----- BNO-055 Initializations -----*/
     memset(_uart_buffer, 0, sizeof(_uart_buffer));
}

BNO055::~BNO055(){
     int err = serial_close(this->_pi, this->_handle);
}

// int BNO055::_imu_write_byte(BNO055Register reg, uint8_t byte){
//      int err = _imu_write_bytes(reg, &byte, 1);
//      return err;
// }
//
// int BNO055::_imu_write_bytes(BNO055Register reg, uint8_t* bytes, int length){
//      int numBytes = (sizeof(bytes)/sizeof(*bytes));
//      printf("[BNO055::_write_bytes] ---- nBytes: %d\r\n", numBytes);
//
//      uint8_t tmp[length+4];
//      tmp[0] = 0xAA;
//      tmp[1] = 0;
//      tmp[2] = (uint8_t)reg;
//      tmp[3] = (unsigned)length;
//      for(int i = 0; i < length; ++i){
// 		// tmp[i+4] = *((uint8_t*)bytes+i);
// 		tmp[i+4] = bytes[i];
// 	}
//
//      int err = _write(&tmp[0], length);
// 	if(err >= 0)
// 		return 0;
// 	else{
//           printf("[BNO055::_write_bytes] ---- ERROR! Unsuccessful write...\r\n");
//           return err;
//      }
//
// }

char* BNO055::_pi_read(int num_bytes, bool verbose){
     if(verbose) printf("[BNO055::_pi_read] ---- # of bytes to read: %d\r\n", num_bytes);

     int nRead = serial_read(_pi,_handle, this->_uart_buffer, num_bytes);
     char tmp[nRead*sizeof(char)];
     memset(tmp, 0, sizeof(tmp));

     if(nRead > 0){
          // tmp = &_uart_buffer[0];
          memcpy(&tmp[0],&_uart_buffer[0],nRead*sizeof(char));
     }else{
          printf("[ERROR] BNO055::_pi_read ---- pigpiod 'serial_read' failed with error code [%d]\r\n", nRead);
          return nullptr;
     }

     if(verbose){
          cout << "[BNO055::_pi_read] ---- bytes received: ";
          for(int i = 0; i < nRead; i++){
               cout << "0x" << std::hex << (int)tmp[i] << ", ";
          }
          cout << endl;
     }
     return tmp;
}

char* BNO055::_uart_send(char* cmds, bool ack, int max_trys){
     int trys = 0;
     int length = (sizeof(cmds)/sizeof(*cmds));
     printf("[BNO055::_uart_send] ---- # of bytes to send: %d\r\n", length);

     while(trys < max_trys + 1){
          // Flush any pending received data to get into a clean state.
          this->flush();
          // Send the data.
          int err = serial_write(_pi,_handle, cmds, length);
          if(err < 0){
               printf("[ERROR] BNO055::_uart_send ---- pigpiod 'serial_write' failed with error code [%d]\r\n", err);
               return nullptr;
          }
          // Stop if no acknowledgment is expected.
          if(!ack) return 0;

          // Read acknowledgement response (2 bytes).
          char* resp = this->_pi_read(2,true);
          int recv_bytes = (sizeof(resp)/sizeof(*resp));
          if((recv_bytes != 2) || (resp == nullptr) ){
               printf("[ERROR] BNO055::_uart_send ---- UART ACK not received, is the BNO055 connected? (HINT: nbytes = %d, or nullptr)\r\n", recv_bytes);
               trys += 1;
               continue;
          }
          // Stop if there's no bus error (0xEE07 response) and return response bytes.
          uint8_t resp_header = (uint8_t)resp[0];
          uint8_t resp_status = (uint8_t)resp[1];
          bool resp_check = ((resp_header == 0xEE) && (resp_status == 0x07) );
          if(!resp_check){
               printf("[INFO] BNO055::_uart_send ---- Response Received (header, status): %#x,\t%#x\r\n", (int)resp_header,(int)resp_status);
               return resp;
          }

          // Else there was a bus error so resend, as recommended in UART app
          trys += 1;
          // printf("[INFO] BNO055::_uart_send ---- No ack recieved, Trying again....\r\n");
     }

     printf("[ERROR] BNO055::_uart_send ---- Exceeded maximum attempts to acknowledge serial command without bus error!\r\n");
     return nullptr;
}

int BNO055::_write_bytes(uint8_t _address, uint8_t* bytes, bool ack){
     int length = (sizeof(bytes)/sizeof(*bytes));
     uint8_t n = (uint8_t)length;
     // Load up Array of bytes for UART <-> BNO-055 register relations
     uint8_t outbytes[4+length];
     outbytes[0] = 0xAA;                // Start byte
     outbytes[1] = 0x00;                // Write
     outbytes[2] = _address & 0xFF;
     outbytes[3] = n & 0xFF;
     for(int i = 0; i < length; ++i){
          uint8_t tmp = bytes[i];
		outbytes[i+4] = tmp & 0xFF;
	}
     // Transmit via UART and get response
     char* resp = this->_uart_send((char*)outbytes, ack);
     // Error Checking
     if(resp == nullptr){
          printf("[ERROR] BNO055::_write_bytes ---- 'NULL'  response received from BNO055::_uart_send\r\n");
          return -1;
     }
     uint8_t resp_header = (uint8_t)resp[0];
     uint8_t resp_status = (uint8_t)resp[1];
     // Verify register write succeeded if there was an acknowledgement.
     if( (resp_header != 0xEE) && (resp_status != 0x01) ){
          printf("[ERROR] BNO055::_write_bytes ---- Could not verify UART ACK (0xEE01). Response Received (header, status): %#x,\t%#x\r\n", (int)resp_header,(int)resp_status);
          return -2;
     }
     return 0;
}

int BNO055::_write_byte(uint8_t _address, uint8_t byte, bool ack){

     // Load up Array of bytes for UART <-> BNO-055 register relations
     uint8_t outbytes[5];
     outbytes[0] = 0xAA;                // Start byte
     outbytes[1] = 0x00;                // Write
     outbytes[2] = _address & 0xFF;
     outbytes[3] = 1;
     outbytes[4] = byte & 0xFF;

     // Transmit via UART and get response
     char* resp = this->_uart_send((char*)outbytes, ack);
     uint8_t resp_header = (uint8_t)resp[0];
     uint8_t resp_status = (uint8_t)resp[1];
     // Verify register write succeeded if there was an acknowledgement.
     if( (ack) && (resp_header != 0xEE) && (resp_status != 0x01) ){
          printf("[ERROR] BNO055::_write_byte ---- Could not verify UART ACK (0xEE01). 'write' Response Received (header, status): %#x,\t%#x\r\n", (int)resp_header,(int)resp_status);
          return -1;
     }
     return 0;
}

int BNO055::_read_bytes(uint8_t _address, int length, uint8_t* recv_data){
     uint8_t n = (uint8_t)length;
     // Load up Array of bytes for UART <-> BNO-055 register relations
     uint8_t outbytes[4];
     outbytes[0] = 0xAA;                // Start byte
     outbytes[1] = 0x01;                // Read
     outbytes[2] = _address & 0xFF;
     outbytes[3] = length & 0xFF;

     // Transmit via UART and get response
     char* resp = this->_uart_send((char*)outbytes);
     uint8_t resp_header = (uint8_t)resp[0];
     uint8_t resp_status = (uint8_t)resp[1];
     int num_bytes_recv = (int)resp[1];
     // Verify register read succeeded.
     if(resp_header != 0xBB){
          printf("[ERROR] BNO055::_read_bytes ---- Could not verify UART ACK (0xBB). 'read' Response Received (header, status): %#x,\t%#x\r\n", (int)resp_header, (int)resp_status);
          return -1;
     }
     printf("[INFO] BNO055::_read_bytes ---- # of bytes received: %d\r\n", num_bytes_recv);
     // Read in the returned bytes
     char* resp2 = this->_pi_read(num_bytes_recv,true);
     int nRead = (sizeof(resp2)/sizeof(*resp2));

     // Error Checking
     if((nRead != num_bytes_recv) || (resp2 == nullptr) ){
          printf("[ERROR] BNO055::_read_bytes ---- UART 'read' mismatching! (HINT: nbytes_read(%d) != nbytes_expected(%d), or response = nullptr)\r\n", nRead,num_bytes_recv);
          return -2;
     }
     // Store
     recv_data = (uint8_t*)resp2;
     return 1;
}

int BNO055::_read_byte(uint8_t _address, uint8_t* recv_data){
     uint8_t* tmp;
     int err = this->_read_bytes(_address, 1, tmp);
     if(err < 0){
          printf("[ERROR] BNO055::_read_byte ---- 'BNO055::_read_bytes' failed with error code [%d]\r\n", err);
          return -1;
     }
     *recv_data = tmp[0];
     return 1;
}

int8_t BNO055::_read_signed_byte(uint8_t _address){
     uint8_t* tmp;
     int8_t out;

     int err = this->_read_byte(_address, tmp);
     if(err > 0){
          memcpy(&out, &tmp[0], sizeof(int8_t));
     }else{
          printf("[ERROR] BNO055::_read_signed_byte ---- 'BNO055::_read_byte' failed with error code [%d]\r\n", err);
          return -1;
     }

     return out;
}


// int BNO055::read(BNO055Register reg, uint8_t *data, int length){
//      uint8_t* buf;
//      uint8_t tmp[4];
//      tmp[0] = 0xAA;
//      tmp[1] = 1;
//      tmp[2] = (uint8_t)reg;
//      tmp[3] = (unsigned)length;
//
//      this->flush();
//      int err = serial_write(_pi,_handle, (char*)tmp, 4);
//      // If bytes successful sent out and ackowledgement received then read in data
// 	if(err >= 0){
//           int nRead = _read(buf,length+2);
//           if(buf[0]==0xBB && buf[1]==length){
//      		memcpy(data,&buf[2],length);
//                return nRead;
//           }
//      }else{
//           printf("[BNO055::read] ---- ERROR! Unsuccessful write... %d\r\n", err);
//           return err;
//      }
// }

int BNO055::available(){
	return serial_data_available(_pi,_handle);
}

void BNO055::flush(){
	char* tmp;
	int buf = available();
	serial_read(_pi,_handle,tmp, buf);
}

int BNO055::begin(BNO055OpMode mode){
     // int err;
     // this->flush();
     // usleep(0.1 * 1000000);
     // if(_imu_write_byte(PAGE0_OPR_MODE,OP_MODE_CONFIG) < 0)
     //      return -1;
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
	return 1;
}

void BNO055::update(){
     // float a[3];
     //
     // int nRead = this->read(PAGE0_ACC_DATA_X_LSB, (uint8_t*)&this->_readings,46);
     //
     // a[0] = ((float)(this->_readings.imu.LinearAccelerationDataX)) / Caccel_fct;
	// a[1] = ((float)(this->_readings.imu.LinearAccelerationDataY)) / Caccel_fct;
	// a[2] = ((float)(this->_readings.imu.LinearAccelerationDataZ)) / Caccel_fct;

     // printf("[BNO055::Accelerometer] ---- X, Y, Z: %.3f\t|\t%.3f\t|\t%.3f\r\n", a[0],a[1],a[2]);

}
