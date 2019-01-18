#include <unistd.h>
#include <iostream>
#include <vector>
#include <string.h>
#include <pigpiod_if2.h>
#include "bno055.h"

using namespace std;

BNO055::BNO055(std::string dev, int baud){
     /** ----- Open UART -----*/
     int err;
     char* device = new char[dev.length()+1];
     strcpy(device, dev.c_str());

     this->_sd = new UartDev(device, baud);
     this->_sd->set_verbosity(false);
     this->_use_pi = false;
     /** ----- BNO-055 Initializations -----*/
     this->_baud = baud;

}

// BNO055::BNO055(int pi, std::string dev, int baud){
//      /** ----- Open UART -----*/
//      int handle;
//      int err;
//      char* device = new char[dev.length()+1];
//      strcpy(device, dev.c_str());
//      if(pi >= 0){
//           this->_pi = pi;
//           handle = serial_open(pi, device, baud,0);
//           if(handle >= 0){
//                this->_handle = handle;
//                this->_baud = baud;
//           }else{
//                printf("[ERROR] BNO-055 at %s with Baud rate of %d could not establish UART connection! Exiting...\r\n",device,baud);
//                this->_initialized = false;
//                std::exit(2);
//           }
//      }else{
//           printf("[ERROR] PigpioD Not properly initialized! Exiting...\r\n");
//           this->_initialized = false;
//           std::exit(1);
//      }
//
//      /** ----- BNO-055 Initializations -----*/
// }

BNO055::~BNO055(){
     this->_sd->_close();
     // serial_close(_pi,_handle);
     delete this->_sd;
}

/*
██████   █████  ███████ ██  ██████
██   ██ ██   ██ ██      ██ ██
██████  ███████ ███████ ██ ██
██   ██ ██   ██      ██ ██ ██
██████  ██   ██ ███████ ██  ██████
*/

int BNO055::_read(int num_bytes, char* data, bool verbose){
     char _buf[num_bytes+10];
     int bytes_read = this->_sd->read_bytes(&_buf[0], num_bytes);
     memcpy(data,_buf, sizeof(char)*bytes_read);
     if(verbose){
          printf("[BNO055::_read] ---- # of bytes to read: %d\r\n", bytes_read);
          for(int i = 0; i < bytes_read; i++){
               std::cout << (int)data[i] << " (0x" << std::hex << (int)data[i] << "), ";
          }
          std::cout << std::endl;
     }
     return bytes_read;
}

int BNO055::_send(char* cmds, int length, char* data, bool ack, int max_trys, bool verbose){
     int trys = 0;
     bool success = false;
     char* _cmds = &cmds[0];

     if(verbose){
          printf("[BNO055::_send] ---- sending [%d] command bytes:", length);
          for(int i = 0; i < length; i++){
               std::cout << (int)_cmds[i] << " (0x" << std::hex << (int)_cmds[i] << "), ";
          }
          std::cout << std::endl;
     }

     while(trys <= max_trys + 1){
          char resp[50];
          // Flush any pending received data to get into a clean state.
          if(verbose) printf("[DEBUG] BNO055::_send ----- Flushing...\n\r");
          this->_sd->flush();

          // Send the data.
          int count = this->_sd->write_bytes(&cmds[0], length);
          usleep(0.03 * 1000000);

          if(verbose) printf("[DEBUG] BNO055::_send ----- commands sent with 'serial_write' with err = %d.\n\r", count);
          if(count < 0){
               printf("[ERROR] BNO055::_send ---- UNIX 'write' failed with error code [%d]\r\n", count);
               break;
          }
          // Stop if no acknowledgment is expected.
          if(!ack){
               if(verbose) printf("[DEBUG] BNO055::_send ----- Not looking for ACK, exiting from 'BNO055::_uart_send'...\n\r");
               success = true;
               break;
          }

          int bytes_avail = this->_sd->bytes_available();

          // Read acknowledgement response (2 bytes).
          if(verbose) printf("[DEBUG] BNO055::_send ----- About to '_read' with [%d] bytes available...\n\r", bytes_avail);
          int nBytes_read = this->_read(bytes_avail,&resp[0]);

          if(verbose){
               printf("[DEBUG] BNO055::_send ----- recieved [%d] bytes from '_read'...\n\r", nBytes_read);
               for(int i = 0; i < nBytes_read; i++){
                    std::cout << (int)resp[i] << " (0x" << std::hex << (int)resp[i] << "), ";
               }
               std::cout << std::endl;
          }

          uint8_t resp_header = (resp[0] & 0xFF);
          uint8_t resp_status = (resp[1] & 0xFF);

          // Stop if there's no bus error (0xEE07 response) and return response bytes.
          bool resp_check = (((int)resp_header == 0xEE) && ((int)resp_status == 0x01) );
          if(resp_check){
               if(verbose) printf("[INFO] BNO055::_send ---- Response Received (header, status): %#x,\t%#x\r\n", (int)resp_header,(int)resp_status);
               memcpy(data,resp, sizeof(char)*nBytes_read);
               success = true;
               break;
          }else if((resp_header == 0xBB)){
               if(verbose) printf("[INFO] BNO055::_send ---- Response Received (header, status): %#x,\t%#x\r\n", (int)resp_header,(int)resp_status);
               memcpy(data,resp, sizeof(char)*nBytes_read);
               success = true;
               break;
          }
          // Else there was a bus error so resend, as recommended in UART app
          trys += 1;
     }

     // Choose what to return
     if(success)
          return 0;
     else{
          printf("[ERROR] BNO055::_send ---- Exceeded maximum attempts to acknowledge serial command without bus error!\r\n");
          return -1;
     }
}

/*
██     ██ ██████  ██ ████████ ███████
██     ██ ██   ██ ██    ██    ██
██  █  ██ ██████  ██    ██    █████
██ ███ ██ ██   ██ ██    ██    ██
 ███ ███  ██   ██ ██    ██    ███████
*/



int BNO055::_write_bytes(uint8_t _address, uint8_t* bytes, int length, bool ack){
     char resp[4096];

     // Load up Array of bytes for UART <-> BNO-055 register relations
     uint8_t outbytes[4+length];
     outbytes[0] = 0xAA;                // Start byte
     outbytes[1] = 0x00;                // Write
     outbytes[2] = _address & 0xFF;
     outbytes[3] = (uint8_t)length & 0xFF;
     for(int i = 0; i < length; ++i){
          uint8_t tmp = bytes[i];
		outbytes[i+4] = tmp & 0xFF;
	}
     // Transmit via UART and get response
     int bytes_sent;
     bytes_sent = this->_send((char*)outbytes, 4+length, &resp[0], ack);
     // if(!this->_use_pi)
     //      bytes_sent = this->_send((char*)outbytes, 4+length, &resp[0], ack);
     // else
     //      bytes_sent = this->_uart_send((char*)outbytes, 4+length, &resp[0], ack);

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
     char resp[4096];
     uint8_t resp_header, resp_status;
     // Load up Array of bytes for UART <-> BNO-055 register relations
     uint8_t outbytes[5];
     outbytes[0] = 0xAA;                // Start byte
     outbytes[1] = 0x00;                // Write
     outbytes[2] = _address & 0xFF;
     outbytes[3] = 1;
     outbytes[4] = byte & 0xFF;

     // Transmit via UART and get response
     int bytes_sent;
     bytes_sent = this->_send((char*)outbytes, 5, &resp[0], ack);
     // if(!this->_use_pi)
     //      bytes_sent = this->_send((char*)outbytes, 5, &resp[0], ack);
     // else
     //      bytes_sent = this->_uart_send((char*)outbytes, 5, &resp[0], ack);

     if(ack){
          if(this->_verbose) printf("[DEBUG] BNO055::_write_byte ---- '_uart_send' Response Received (header, status): %#x,\t%#x\r\n", (int)resp[0],(int)resp[1]);
          resp_header = (int)resp[0];
          resp_status = (int)resp[1];
     }
     // Verify register write succeeded if there was an acknowledgement.
     if( (ack) && (resp_header != 0xEE) && (resp_status != 0x01) ){
          printf("[ERROR] BNO055::_write_byte ---- Could not verify UART ACK (0xEE01). 'write' Response Received (header, status): %#x,\t%#x\r\n", (int)resp_header,(int)resp_status);
          return -1;
     }
     return 0;
}

/*
██████  ███████  █████  ██████
██   ██ ██      ██   ██ ██   ██
██████  █████   ███████ ██   ██
██   ██ ██      ██   ██ ██   ██
██   ██ ███████ ██   ██ ██████
*/

int BNO055::_read_bytes(uint8_t _address, int length, uint8_t* recv_data){
     char resp[4096];
     char resp2[4096];
     // Load up Array of bytes for UART <-> BNO-055 register relations
     uint8_t outbytes[4];
     outbytes[0] = 0xAA;                // Start byte
     outbytes[1] = 0x01;                // Read
     outbytes[2] = _address & 0xFF;
     outbytes[3] = length & 0xFF;

     // Transmit via UART and get response
     int bytes_sent;
     bytes_sent = this->_send((char*)outbytes, 4, &resp[0]);
     // if(!this->_use_pi)
     //      bytes_sent = this->_send((char*)outbytes, 4, &resp[0]);
     // else
     //      bytes_sent = this->_uart_send((char*)outbytes, 4, &resp[0]);

     uint8_t resp_header = (uint8_t)resp[0] ;
     uint8_t resp_status = (uint8_t)resp[1];
     if(this->_verbose) printf("[INFO] BNO055::_read_bytes ---- Response Received (header, status): %#x,\t%#x\r\n", (int)resp_header,(int)resp_status);
     // Verify register read succeeded.
     if(resp_header != 0xBB){
          printf("[ERROR] BNO055::_read_bytes ---- Could not verify UART ACK (0xBB). 'read' Response Received (header, status): %#x,\t%#x\r\n", (int)resp_header, (int)resp_status);
          return -1;
     }
     memcpy(recv_data, &resp[2],sizeof(uint8_t)*length);
     return (int)resp_status;
}

int BNO055::_read_byte(uint8_t _address, uint8_t* recv_data){
     uint8_t tmp[1];
     int err = this->_read_bytes(_address, 1, &tmp[0]);
     if(err < 0){
          printf("[ERROR] BNO055::_read_byte ---- 'BNO055::_read_bytes' failed with error code [%d]\r\n", err);
          return -1;
     }
     memcpy(recv_data, tmp,sizeof(char));
     return 1;
}

int8_t BNO055::_read_signed_byte(uint8_t _address){
     int8_t out;
     int err = this->_read_byte(_address, (uint8_t*)&out);
     if(err < 0){
          printf("[ERROR] BNO055::_read_signed_byte ---- 'BNO055::_read_byte' failed with error code [%d]\r\n", err);
          return -1;
     }
     if(out > 127) out -= 256;
     return out;
}

int BNO055::read_vector(uint8_t _address, int16_t* data, int count){
     int nBytes = count*2;
     char buf[nBytes];
     uint16_t byte_array[count];

     int nRead = this->_read_bytes(_address, nBytes, (uint8_t*)&buf[0]);

     for(int i = 0; i < count; i++){
          uint8_t byte_msb = (int)buf[i*2+1];
          uint8_t byte_lsb = (int)buf[i*2];
          uint16_t tmp_byte = byte_msb << 8 | (byte_lsb & 0xFF);
          int tmp_val = (int)tmp_byte;
          if(tmp_byte > 32767) uint16_t new_byte = tmp_byte - 65536;
          byte_array[i] = (int16_t) tmp_val;
     }
     memcpy(data,byte_array, sizeof(int16_t) * count);
     return 1;
}

/*
███████ ██████  ███████  ██████ ██ ███████ ██  ██████
██      ██   ██ ██      ██      ██ ██      ██ ██
███████ ██████  █████   ██      ██ █████   ██ ██
     ██ ██      ██      ██      ██ ██      ██ ██
███████ ██      ███████  ██████ ██ ██      ██  ██████
*/

void BNO055::_config_mode(){this->set_mode(OPERATION_MODE_CONFIG);}
void BNO055::_operation_mode(){this->set_mode(this->_mode);}

/*
███████ ███████ ████████ ████████ ███████ ██████  ███████
██      ██         ██       ██    ██      ██   ██ ██
███████ █████      ██       ██    █████   ██████  ███████
     ██ ██         ██       ██    ██      ██   ██      ██
███████ ███████    ██       ██    ███████ ██   ██ ███████
*/

int BNO055::set_mode(uint8_t mode){
     int err = this->_write_byte(BNO055_OPR_MODE_ADDR, mode & 0xFF);
     usleep(0.03 * 1000000);
     return err;
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

/** ==================================================================
*						    TODO
* ==================================================================== */

/** REFERENCE:
* https://github.com/adafruit/Adafruit_Python_BNO055/blob/master/Adafruit_BNO055/BNO055.py
*/
int BNO055::set_calibration(uint8_t* data){return 0;}
int BNO055::set_axis_remap(int x, int y, int z, int xsign, int ysign, int zsign){return 0;}

/** ==================================================================
*						   END TODO
* ==================================================================== */

/*
██████  ███████  ██████  ██ ███    ██
██   ██ ██      ██       ██ ████   ██
██████  █████   ██   ███ ██ ██ ██  ██
██   ██ ██      ██    ██ ██ ██  ██ ██
██████  ███████  ██████  ██ ██   ████
*/

int BNO055::begin(uint8_t mode){
     printf(" ============ BNO055::begin ============r\n");
     int err;
     uint8_t bnoId;
     this->_mode = mode;
     // printf(" ---------- Byte #1\r\n");
     this->_write_byte(BNO055_PAGE_ID_ADDR, 0, false);
     // printf(" ---------- Config Mode\r\n");
     this->_config_mode();
     // printf(" ---------- Byte #2\r\n");
     this->_write_byte(BNO055_PAGE_ID_ADDR, 0);
     // printf(" ---------- Chip ID\r\n");
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
     // printf(" ---------- Normal PWR Mode\r\n");
     this->_write_byte(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
     // Default to internal oscillator.
     // printf(" ---------- Defaulting to Internal Oscillator\r\n");
     this->_write_byte(BNO055_SYS_TRIGGER_ADDR, 0x0);
     // Enter normal operation mode.
     // printf(" ---------- Normal Operation Mode\r\n");
     this->_operation_mode();

     return 1;
}

/*
 ██████  ███████ ████████ ████████ ███████ ██████  ███████
██       ██         ██       ██    ██      ██   ██ ██
██   ███ █████      ██       ██    █████   ██████  ███████
██    ██ ██         ██       ██    ██      ██   ██      ██
 ██████  ███████    ██       ██    ███████ ██   ██ ███████
*/

void BNO055::get_revision(int* revision_data){
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
     memcpy(revision_data,tmp,5*sizeof(int));
}

void BNO055::get_system_status(int* status, bool run_self_test){
     uint8_t sys_trigger, _status, error;
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
     this->_read_byte(BNO055_SYS_STAT_ADDR, &_status);
     this->_read_byte(BNO055_SYS_ERR_ADDR, &error);

     int tmp[3] = {(int)_status, (int)self_test, (int)error};
     memcpy(status,tmp,3*sizeof(int));
}

/** ==================================================================
*						    TODO
* ==================================================================== */

/** REFERENCE:
* https://github.com/adafruit/Adafruit_Python_BNO055/blob/master/Adafruit_BNO055/BNO055.py
*/
void BNO055::get_calibration_status(int* status){}
void BNO055::get_calibration(float* data){}
void BNO055::get_axis_remap(int* data){}
/** ==================================================================
*						   END TODO
* ==================================================================== */


void BNO055::get_euler(float* data, bool verbose){
     int16_t _data[3];
     float tmp[3];
     this->read_vector(BNO055_EULER_H_LSB_ADDR, &_data[0]);
     /** Re-arrange reutrned array as R, P, Y b/c BNO-055 returns Y, R, P */
     tmp[2] = (float) _data[0] / 16.0;
     tmp[0] = (float) _data[1] / 16.0;
     tmp[1] = (float) _data[2] / 16.0;
     if(verbose) printf(" Euler Angles: %f, %f, %f\r\n", tmp[0], tmp[1] , tmp[2]);
     memcpy(data,tmp, sizeof(tmp));
}

void BNO055::get_magnetometer(float* data, bool verbose){
     int16_t _data[3];
     float tmp[3];
     this->read_vector(BNO055_MAG_DATA_X_LSB_ADDR, &_data[0]);
     tmp[0] = (float) _data[0] / 16.0;
     tmp[1] = (float) _data[1] / 16.0;
     tmp[2] = (float) _data[2] / 16.0;
     if(verbose) printf("Magnetometer (micro-Teslas) X, Y, Z: %f, %f, %f\r\n", tmp[0], tmp[1] , tmp[2]);
     memcpy(data,tmp, sizeof(tmp));
}

void BNO055::get_gyroscope(float* data, bool verbose){
     int16_t _data[3];
     float tmp[3];
     this->read_vector(BNO055_GYRO_DATA_X_LSB_ADDR, &_data[0]);
     tmp[0] = (float) _data[0] / 900.0;
     tmp[1] = (float) _data[1] / 900.0;
     tmp[2] = (float) _data[2] / 900.0;
     if(verbose) printf("Gyroscope (rad/sec) X, Y, Z: %f, %f, %f\r\n", tmp[0], tmp[1] , tmp[2]);
     memcpy(data,tmp, sizeof(tmp));
}

void BNO055::get_accelerometer(float* data, bool verbose){
     int16_t _data[3];
     float tmp[3];
     this->read_vector(BNO055_ACCEL_DATA_X_LSB_ADDR, &_data[0]);
     tmp[0] = (float) _data[0] / 100.0;
     tmp[1] = (float) _data[1] / 100.0;
     tmp[2] = (float) _data[2] / 100.0;
     if(verbose) printf("Accelerometer (m/s^2) X, Y, Z: %f, %f, %f\r\n", tmp[0], tmp[1] , tmp[2]);
     memcpy(data,tmp, sizeof(tmp));
}

void BNO055::get_linear_acceleration(float* data, bool verbose){
     int16_t _data[3];
     float tmp[3];
     this->read_vector(BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, &_data[0]);
     tmp[0] = (float) _data[0] / 100.0;
     tmp[1] = (float) _data[1] / 100.0;
     tmp[2] = (float) _data[2] / 100.0;
     if(verbose) printf("Linear Acceleration (m/s^2) X, Y, Z: %f, %f, %f\r\n", tmp[0], tmp[1] , tmp[2]);
     memcpy(data,tmp, sizeof(tmp));
}

void BNO055::get_gravity(float* data, bool verbose){
     int16_t _data[3];
     float tmp[3];
     this->read_vector(BNO055_GRAVITY_DATA_X_LSB_ADDR, &_data[0]);
     tmp[0] = (float) _data[0] / 100.0;
     tmp[1] = (float) _data[1] / 100.0;
     tmp[2] = (float) _data[2] / 100.0;
     if(verbose) printf("Gravity (m/s^2) X, Y, Z: %f, %f, %f\r\n", tmp[0], tmp[1] , tmp[2]);
     memcpy(data,tmp, sizeof(tmp));
}

void BNO055::get_quaternions(float* data, bool verbose){
     int16_t _data[4];
     float tmp[4];
     float scale = (1.0 / (1<<14));
     this->read_vector(BNO055_QUATERNION_DATA_W_LSB_ADDR, &_data[0], 4);
     tmp[0] = (float) _data[0] * scale;
     tmp[1] = (float) _data[1] * scale;
     tmp[2] = (float) _data[2] * scale;
     tmp[3] = (float) _data[3] * scale;
     if(verbose) printf("Quaternion X, Y, Z, W: %f, %f, %f, %f\r\n", tmp[0], tmp[1], tmp[2], tmp[3]);
     memcpy(data,tmp, sizeof(tmp));
}

float BNO055::get_tempurature(bool verbose){
     int8_t data = this->_read_signed_byte(BNO055_TEMP_ADDR);
     if(verbose) printf("Temperature [Celsius]: %f\r\n", (float)data);
     return (float)data;
}

void BNO055::update(bool verbose){
     float angs[3]; float accel[3]; float gyros[3]; float mags[3];
     float lin_accel[3]; float grav[3]; float quats[4];
     if(verbose) printf("==========================\r\n");
     this->get_euler(&angs[0], verbose);
     this->get_magnetometer(&mags[0], verbose);
     this->get_gyroscope(&gyros[0], verbose);
     this->get_accelerometer(&accel[0], verbose);
     this->get_linear_acceleration(&lin_accel[0], verbose);
     this->get_gravity(&grav[0], verbose);
     this->get_quaternions(&quats[0], verbose);
     float temp = this->get_tempurature(verbose);
}

/*
██████  ██
██   ██ ██
██████  ██
██      ██
██      ██
*/

// int BNO055::_pi_read(int num_bytes, char* data, bool verbose){
//      char buffer[num_bytes];
//      int nRead = serial_read(_pi,_handle, buffer, num_bytes);
//
//      if(nRead >= 0){
//           memcpy(data,buffer, sizeof(char)*nRead);
//      }else{
//           printf("[ERROR] BNO055::_pi_read ---- pigpiod 'serial_read' failed with error code [%d]\r\n", nRead);
//           return -1;
//      }
//
//      if(verbose){
//           cout << "[BNO055::_pi_read] ---- bytes received [N = " << nRead <<"]: ";
//           for(int i = 0; i <= nRead; i++){
//                cout << (int)buffer[i] << " (0x" << std::hex << (int)buffer[i] << ") , ";
//           }
//           cout << endl;
//      }
//      return nRead;
// }
//
// int BNO055::_uart_send(char* cmds, int length, char* data, bool ack, bool verbose, int max_trys){
//      char* output;
//      char* resp;
//      bool success = false;
//      int trys = 0;
//      char* _cmds = &cmds[0];
//      if(verbose){
//           printf("[BNO055::_uart_send] ---- # of bytes to send: %d\r\n", length);
//           cout << "[BNO055::_uart_send] ---- command bytes sending: ";
//           for(int i = 0; i <= length; i++){
//                cout << (int)_cmds[i] << " (0x" << std::hex << (int)_cmds[i] << "), ";
//           }
//           cout << endl;
//      }
//
//      while(trys <= max_trys + 1){
//           char resp[50];
//           // Flush any pending received data to get into a clean state.
//           if(verbose) printf("[DEBUG] BNO055::_uart_send ----- Flushing...\n\r");
//           this->_flush();
//           // Send the data.
//           int err = serial_write(_pi,_handle, &cmds[0], length);
//           usleep(0.03 * 1000000);
//           if(verbose) printf("[DEBUG] BNO055::_uart_send ----- commands sent with 'serial_write' with err = %d.\n\r", err);
//           if(err < 0){
//                printf("[ERROR] BNO055::_uart_send ---- pigpiod 'serial_write' failed with error code [%d]\r\n", err);
//                break;
//           }
//           // Stop if no acknowledgment is expected.
//           if(!ack){
//                if(verbose) printf("[DEBUG] BNO055::_uart_send ----- Not looking for ACK, exiting from 'BNO055::_uart_send'...\n\r");
//                success = true;
//                break;
//           }
//
//           // Read acknowledgement response (2 bytes).
//           int bytes_avail = this->_available();
//           int nBytes_read = this->_pi_read(bytes_avail,&resp[0]);
//
//           if(verbose) printf("[INFO] BNO055::_uart_send ---- Response Received (header, status): %#x,\t%#x\r\n", (int)resp[0],(int)resp[1]);
//
//           // Stop if there's no bus error (0xEE07 response) and return response bytes.
//           uint8_t resp_header = (resp[0] & 0xFF);
//           uint8_t resp_status = (resp[1] & 0xFF);
//           bool resp_check = (((int)resp_header == 0xEE) && ((int)resp_status == 0x01) );
//           if(resp_check){
//                if(verbose) printf("[INFO] BNO055::_uart_send ---- Response Received (header, status): %#x,\t%#x\r\n", (int)resp_header,(int)resp_status);
//                memcpy(data,resp, sizeof(char)*nBytes_read);
//                success = true;
//                break;
//           }else if((resp_header == 0xBB)){
//                if(verbose) printf("[INFO] BNO055::_uart_send ---- Response Received (header, status): %#x,\t%#x\r\n", (int)resp_header,(int)resp_status);
//                memcpy(data,resp, sizeof(char)*nBytes_read);
//                success = true;
//                break;
//           }
//
//           // Else there was a bus error so resend, as recommended in UART app
//           trys += 1;
//           // printf("[INFO] BNO055::_uart_send ---- No ack recieved, Trying again....\r\n");
//      }
//
//      // Choose what to return
//      if(success)
//           return 0;
//      else{
//           printf("[ERROR] BNO055::_uart_send ---- Exceeded maximum attempts to acknowledge serial command without bus error!\r\n");
//           return -1;
//      }
// }
// void BNO055::_flush(){
// 	char tmp[4096];
// 	int buf = _available();
// 	serial_read(_pi,_handle,tmp, buf);
// }
//
// int BNO055::_available(){
// 	return serial_data_available(_pi,_handle);
// }
