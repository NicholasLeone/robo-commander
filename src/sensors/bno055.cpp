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

int BNO055::available(){
	return serial_data_available(_pi,_handle);
}

void BNO055::flush(){
	char* tmp;
	int buf = available();
	serial_read(_pi,_handle,tmp, buf);
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
     if(verbose) printf("[BNO055::_pi_read] ---- num_bytes, available: %d, %d\r\n", num_bytes, this->available());
     char* buffer;
     // char* tmp;
     int nRead = serial_read(_pi,_handle, buffer, num_bytes);

     if(nRead >= 0){
          // memcpy(tmp,&buffer[0],nRead*sizeof(char));
     }else{
          printf("[ERROR] BNO055::_pi_read ---- pigpiod 'serial_read' failed with error code [%d]\r\n", nRead);
          return nullptr;
     }

     if(verbose){
          cout << "[BNO055::_pi_read] ---- bytes received [N = " << nRead <<"]: ";
          for(int i = 0; i <= nRead; i++){
               cout << (int)buffer[i] << " (0x" << std::hex << (int)buffer[i] << ") , ";
          }
          cout << endl;
     }
     return buffer;
}

char* BNO055::_uart_send(char* cmds, bool ack, bool verbose, int max_trys){
     char* output;
     bool success = false;
     int trys = 0;
     int length = sizeof(cmds) / sizeof(*cmds);
     char* _cmds = &cmds[0];
     if(verbose){
          printf("[BNO055::_uart_send] ---- # of bytes to send: %d\r\n", length);
          cout << "[BNO055::_uart_send] ---- command bytes sending: ";
          for(int i = 0; i <= length; i++){
               cout << (int)_cmds[i] << " (0x" << std::hex << (int)_cmds[i] << "), ";
          }
          cout << endl;
     }

     while(trys <= max_trys + 1){
          // Flush any pending received data to get into a clean state.
          if(verbose) printf("[DEBUG] BNO055::_uart_send ----- Flushing...\n\r");
          this->flush();
          // Send the data.
          if(verbose) printf("[DEBUG] BNO055::_uart_send ----- sending commands with 'serial_write'...\n\r");
          int err = serial_write(_pi,_handle, _cmds, length+1);
          if(verbose) printf("[DEBUG] BNO055::_uart_send ----- commands sent with 'serial_write' with err = %d.\n\r", err);
          if(err < 0){
               printf("[ERROR] BNO055::_uart_send ---- pigpiod 'serial_write' failed with error code [%d]\r\n", err);
               break;
          }
          // Stop if no acknowledgment is expected.
          if(!ack){
               if(verbose) printf("[DEBUG] BNO055::_uart_send ----- Not looking for ACK, exiting from 'BNO055::_uart_send'...\n\r");
               success = true;
               output = nullptr;
               break;
          }

          // Read acknowledgement response (2 bytes).
          if(verbose) printf("[DEBUG] BNO055::_uart_send ----- About to '_pi_read'...\n\r");
          char* resp = this->_pi_read(2,true);
          int recv_bytes = (sizeof(resp)/sizeof(*resp));
          if(verbose) printf("[DEBUG] BNO055::_uart_send ----- recieved [%d] bytes from '_pi_read'...\n\r", recv_bytes);
          printf("[INFO] BNO055::_uart_send ---- Response Received (header, status): %#x,\t%#x\r\n", (int)resp[0],(int)resp[1]);
          if((recv_bytes != 2) || (resp == nullptr) ){
               printf("[ERROR] BNO055::_uart_send ---- UART ACK not received, is the BNO055 connected? (HINT: nbytes = %d, or nullptr)\r\n", recv_bytes);
          }
          // Stop if there's no bus error (0xEE07 response) and return response bytes.
          uint8_t resp_header = (int)resp[0];
          uint8_t resp_status = (int)resp[1];
          bool resp_check = ((resp_header == 0xEE) && (resp_status == 0x07) );
          if(!resp_check){
               printf("[INFO] BNO055::_uart_send ---- Response Received (header, status): %#x,\t%#x\r\n", (int)resp_header,(int)resp_status);
               output = resp;
               success = true;
               break;
          }

          // Else there was a bus error so resend, as recommended in UART app
          trys += 1;
          // printf("[INFO] BNO055::_uart_send ---- No ack recieved, Trying again....\r\n");
     }

     // Choose what to return
     if(success)
          return output;
     else{
          printf("[ERROR] BNO055::_uart_send ---- Exceeded maximum attempts to acknowledge serial command without bus error!\r\n");
          return nullptr;
     }
}

int BNO055::_write_bytes(uint8_t _address, uint8_t* bytes, bool ack){
     int length = sizeof(bytes) / sizeof(bytes[0]);
     uint8_t n = (uint8_t)length;
     printf("[DEBUG] 'BNO055::_write_bytes' ----- # of bytes going out (length, n): %d, %d\r\n", length, (int)n);
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
     char* resp;
     uint8_t resp_header, resp_status;
     // Load up Array of bytes for UART <-> BNO-055 register relations
     uint8_t outbytes[5];
     outbytes[0] = 0xAA;                // Start byte
     outbytes[1] = 0x00;                // Write
     outbytes[2] = _address & 0xFF;
     outbytes[3] = 1;
     outbytes[4] = byte & 0xFF;

     // Transmit via UART and get response
     printf("[DEBUG] BNO055::_write_byte ---- sending byte out using '_uart_send'...\r\n");
     resp = this->_uart_send((char*)outbytes, ack);
     if(ack){
          printf("[DEBUG] BNO055::_write_byte ---- '_uart_send' Response Received (header, status): %#x,\t%#x\r\n", (int)resp[0],(int)resp[1]);
          resp_header = (int)resp[0];
          resp_status = (int)resp[1];
          printf("[DEBUG] BNO055::_write_byte ---- Verifying proper ACK...\r\n");
     }
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
     // recv_data = (uint8_t*)resp2;
     memcpy(recv_data, &resp2[0],sizeof(resp2));
     return 1;
}

int BNO055::_read_byte(uint8_t _address, uint8_t* recv_data){
     uint8_t* tmp;
     int err = this->_read_bytes(_address, 1, tmp);
     if(err < 0){
          printf("[ERROR] BNO055::_read_byte ---- 'BNO055::_read_bytes' failed with error code [%d]\r\n", err);
          return -1;
     }
     memcpy(recv_data, &tmp[0],sizeof(tmp));
     return 1;
}

int8_t BNO055::_read_signed_byte(uint8_t _address){
     uint8_t tmp;
     int8_t out;
     int err = this->_read_byte(_address, &tmp);
     if(err > 0){
          memcpy(&out, &tmp, sizeof(int8_t));
     }else{
          printf("[ERROR] BNO055::_read_signed_byte ---- 'BNO055::_read_byte' failed with error code [%d]\r\n", err);
          return -1;
     }

     return out;
}

void BNO055::_config_mode(){this->set_mode(OPERATION_MODE_CONFIG);}
void BNO055::_operation_mode(){this->set_mode(this->_mode);}

int BNO055::set_mode(uint8_t mode){
     int err = this->_write_byte(BNO055_OPR_MODE_ADDR, mode & 0xFF);
     usleep(0.03 * 1000000);
     return err;
}

/**
* @desc: Returns the following revision information about the BNO055 chip.
*
*    @return[0]: Software revision      - 2 bytes (MSB + LSB)
*    @return[1]: Bootloader version     - 1 byte
*    @return[2]: Accelerometer ID       - 1 byte
*    @return[4]: Gyro ID                - 1 byte
*    @return[3]: Magnetometer ID        - 1 byte
*/
int* BNO055::get_revision(){
     // Initialize bytes
     uint8_t sw_msb, sw_lsb, blId, accelId, gyroId, magId;
     int* out;

     // Read registers
     this->_read_byte(BNO055_SW_REV_ID_MSB_ADDR, &sw_msb);
     this->_read_byte(BNO055_SW_REV_ID_LSB_ADDR, &sw_lsb);
     this->_read_byte(BNO055_BL_REV_ID_ADDR, &blId);
     this->_read_byte(BNO055_ACCEL_REV_ID_ADDR, &accelId);
     this->_read_byte(BNO055_GYRO_REV_ID_ADDR, &gyroId);
     this->_read_byte(BNO055_MAG_REV_ID_ADDR, &magId);

     uint16_t swId = ((sw_msb << 8) | sw_lsb) & 0xFFFF;

     int tmp[5] = {(int)swId, (int)blId, (int)accelId, (int)gyroId, (int)magId};
     memcpy(out,&tmp[0],5*sizeof(int));
     return out;
}

/**
* @desc: Return a tuple with status information.  Three values will be returned:
*
*    @output - System status register value with the following meaning:
*         0 = Idle
*         1 = System Error
*         2 = Initializing Peripherals
*         3 = System Initialization
*         4 = Executing Self-Test
*         5 = Sensor fusion algorithm running
*         6 = System running without fusion algorithms
*    @output - Self test result register value with the following meaning:
*         Bit value: 1 = test passed, 0 = test failed
*         Bit 0 = Accelerometer self test
*         Bit 1 = Magnetometer self test
*         Bit 2 = Gyroscope self test
*         Bit 3 = MCU self test
*         Value of 0x0F = all good!
*    @output - System error register value with the following meaning:
*         0 = No error
*         1 = Peripheral initialization error
*         2 = System initialization error
*         3 = Self test result failed
*         4 = Register map value out of range
*         5 = Register map address out of range
*         6 = Register map write error
*         7 = BNO low power mode not available for selected operation mode
*         8 = Accelerometer power mode not available
*         9 = Fusion algorithm configuration error
*         10 = Sensor configuration error
*
* If run_self_test is passed in as False then no self test is performed and
* None will be returned for the self test result.  Note that running a
* self test requires going into config mode which will stop the fusion
* engine from running.
*/
int* BNO055::get_system_status(bool run_self_test){
     uint8_t sys_trigger, status, error;
     uint8_t self_test = -1;
     int* out;

     if(run_self_test){
          // Switch to configuration mode if running self test.
          this->_config_mode();
          // Perform a self test.
          this->_read_byte(BNO055_SYS_TRIGGER_ADDR, &sys_trigger);
          this->_write_byte(BNO055_SYS_TRIGGER_ADDR, sys_trigger | 0x1);
          // Wait for self test to finish.
          usleep(1000000);
          // Read test result.
          this->_read_byte(BNO055_SELFTEST_RESULT_ADDR, &self_test);
          // Go back to operation mode.
          this->_operation_mode();
     }
     // Now read status and error registers.
     this->_read_byte(BNO055_SYS_STAT_ADDR, &status);
     this->_read_byte(BNO055_SYS_ERR_ADDR, &error);

     int tmp[3] = {(int)status, (int)self_test, (int)error};
     memcpy(out,&tmp[0],3*sizeof(int));
     return out;
}

void BNO055::set_external_crystal(bool use_external_crystal){
     // Switch to configuration mode.
     this->_config_mode();
     if(use_external_crystal)
          this->_write_byte(BNO055_SYS_TRIGGER_ADDR, 0x80);
     else
          this->_write_byte(BNO055_SYS_TRIGGER_ADDR, 0x00);
     // Go back to normal operation mode.
     this->_operation_mode();
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



int BNO055::begin(uint8_t mode){
     printf(" ============ BNO055::begin ============r\n");
     int err;
     uint8_t bnoId;
     this->_mode = mode;
     printf(" ---------- Byte #1\r\n");
     this->_write_byte(BNO055_PAGE_ID_ADDR, 0, false);
     printf(" ---------- Config Mode\r\n");
     this->_config_mode();
     printf(" ---------- Byte #2\r\n");
     this->_write_byte(BNO055_PAGE_ID_ADDR, 0);
     printf(" ---------- Chip ID\r\n");
     this->_read_byte(BNO055_CHIP_ID_ADDR, &bnoId);
     printf("[INFO] BNO055::begin ---- Read BNO-055 Chip ID: %#x\r\n", bnoId);

     // Mis-matching Chip ID's = Failure to begin
     if(bnoId != BNO055_ID)
          return -1;
     // Reset Device
     printf(" ---------- Resetting\r\n");
     this->_write_byte(BNO055_SYS_TRIGGER_ADDR, 0x20, false);
     usleep(0.65 * 1000000);

     // Set to normal power mode.
     printf(" ---------- Normal PWR Mode\r\n");
     this->_write_byte(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
     // Default to internal oscillator.
     printf(" ---------- Defaulting to Internal Oscillator\r\n");
     this->_write_byte(BNO055_SYS_TRIGGER_ADDR, 0x0);
     // Enter normal operation mode.
     printf(" ---------- Normal Operation Mode\r\n");
     this->_operation_mode();

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
