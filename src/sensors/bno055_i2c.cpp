#include <unistd.h>      // For usleep
#include <iostream>      // For printf
#include <string.h>      // For memcpy
#include "sensors/bno055_i2c.h"

using namespace std;

// BNO055_I2C::BNO055_I2C(int dev, int bus, int address) : I2C(dev,bus,address){
//      I2C::_handle = attachPeripheral(I2C_PI, bus, address);
//      this->_mux_channel = 0;
//      this->_mux = nullptr;
// }

BNO055_I2C::BNO055_I2C(int dev, int bus, int address, int channel, TCA9548A* mux) : I2C(dev,bus,address){
     this->_mux_channel = channel;
     this->_mux = mux;
}

BNO055_I2C::~BNO055_I2C(){}

/*
███    ███  █████   ██████ ██████   ██████  ███████
████  ████ ██   ██ ██      ██   ██ ██    ██ ██
██ ████ ██ ███████ ██      ██████  ██    ██ ███████
██  ██  ██ ██   ██ ██      ██   ██ ██    ██      ██
██      ██ ██   ██  ██████ ██   ██  ██████  ███████
*/

int BNO055_I2C::begin(uint8_t mode, bool debug){
     bool isReady = this->isMuxChannelSelected();
     if(debug) printf(" ============ BNO055_I2C::begin ============r\n");
     this->_mode = mode;
     if(debug) printf("[INFO] BNO055_I2C::begin ---- Byte #1\r\n");
     this->write_byte(BNO055_PAGE_ID_ADDR, 0, false);
     if(debug) printf("[INFO] BNO055_I2C::begin ---- Config Mode\r\n");
     this->_config_mode();
     if(debug) printf("[INFO] BNO055_I2C::begin ---- Byte #2\r\n");
     this->write_byte(BNO055_PAGE_ID_ADDR, 0);
     if(debug) printf("[INFO] BNO055_I2C::begin ---- Chip ID\r\n");
     uint8_t bnoId = this->read_raw_byte(BNO055_CHIP_ID_ADDR);
     if(debug) printf("[INFO] BNO055_I2C::begin ---- Read BNO-055 Chip ID: %#x\r\n", bnoId);
     // Mis-matching Chip ID's = Failure to begin
     if(bnoId != BNO055_ID) return -1;

     // Reset Device
     if(debug) printf("[INFO] BNO055_I2C::begin ---- Resetting\r\n");
     this->write_byte(BNO055_SYS_TRIGGER_ADDR, 0x20);
     usleep(0.65 * 1000000);

     // Set to normal power mode.
     if(debug) printf("[INFO] BNO055_I2C::begin ---- Normal PWR Mode\r\n");
     this->write_byte(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
     // Default to internal oscillator.
     if(debug) printf("[INFO] BNO055_I2C::begin ---- Defaulting to Internal Oscillator\r\n");
     this->write_byte(BNO055_SYS_TRIGGER_ADDR, 0x0);
     // Enter normal operation mode.
     if(debug) printf("[INFO] BNO055_I2C::begin ---- Normal Operation Mode\r\n");
     this->_operation_mode();
     return 1;
}

void BNO055_I2C::update(bool getAccel, bool getGyro, bool getMag, bool getAngles, \
                        bool getQuats, bool getTemp, bool getLinAccel, \
                        bool getGravity, bool verbose){
     float angs[3] = {0.0, 0.0, 0.0};
     float accel[3] = {0.0, 0.0, 0.0};
     float gyros[3] = {0.0, 0.0, 0.0};
     float mags[3] = {0.0, 0.0, 0.0};
     float lin_accel[3] = {0.0, 0.0, 0.0};
     float grav[3] = {0.0, 0.0, 0.0};
     float quats[4] = {0.0, 0.0, 0.0, 0.0};
     float temp = 0;
     if(verbose) printf("==========================\r\n");
     if(getAngles) this->get_euler(&angs[0], verbose);
     if(getMag) this->get_magnetometer(&mags[0], verbose);
     if(getGyro) this->get_gyroscope(&gyros[0], verbose);
     if(getAccel) this->get_accelerometer(&accel[0], verbose);
     if(getLinAccel) this->get_linear_acceleration(&lin_accel[0], verbose);
     if(getGravity) this->get_gravity(&grav[0], verbose);
     if(getQuats) this->get_quaternions(&quats[0], verbose);
     if(getTemp) temp = this->get_temperature(verbose);
}

int BNO055_I2C::startup(bool verbose){
     int imu_status[3];
	int imu_revision[5];
	int imu_cal[11];
	int imu_axis[6];

     // Initialize IMU
     int err = this->begin();
	if(err < 0){ printf("[ERROR] BNO055::begin] ---- %d.\r\n", err); }
	else printf("[SUCCESS] BNO-055 Initialized \r\n");

     // Check IMU States
	this->get_system_status(&imu_status[0]);
     this->get_revision(&imu_revision[0]);
     this->get_calibration(&imu_cal[0]);
     this->get_axis_remap(&imu_axis[0]);
     if(verbose){
     	printf("===========    BNO-055 Status Response    =================\r\n");
          printf("BNO-055 Status:\r\n");
          printf(" ------------- \r\n");
          printf("\tSystem Status: %d (%#x)\r\n", imu_status[0], imu_status[0]);
          printf("\tSelf-Test Result: %d (%#x)\r\n", imu_status[1], imu_status[1]);
          printf("\tSystem Error Status: %d (%#x)\r\n", imu_status[2], imu_status[2]);
          printf("BNO-055 Software Revision:\r\n");
          printf(" ------------------------ \r\n");
          printf("\tSoftware Version: %d (%#x)\r\n", imu_revision[0], imu_revision[0]);
          printf("\tBootloader Version: %d (%#x)\r\n", imu_revision[1], imu_revision[1]);
          printf("\tAccelerometer ID: %d (%#x)\r\n", imu_revision[2], imu_revision[2]);
          printf("\tGyroscope ID: %d (%#x)\r\n", imu_revision[3], imu_revision[3]);
          printf("\tMagnetometer ID: %d (%#x)\r\n", imu_revision[4], imu_revision[4]);
     	printf("BNO-055 Calibration Values:\r\n");
          printf(" ------------------------- \r\n");
     	printf("\tAccelerometer Offsets (x,y,z,r): %d, %d, %d, %d\r\n",imu_cal[0],imu_cal[1],imu_cal[2],imu_cal[9]);
     	printf("\tMagnetometer Offsets (x,y,z,r): %d, %d, %d, %d\r\n",imu_cal[3],imu_cal[4],imu_cal[5],imu_cal[10]);
     	printf("\tGyroscope Offsets (x,y,z): %d, %d, %d\r\n",imu_cal[6],imu_cal[7],imu_cal[8]);
     	printf("BNO-055 Axis Mappings:\r\n");
          printf(" -------------------- \r\n");
          printf("\tX-Axis Map (Sign): %d (%d)\r\n", imu_axis[0], imu_axis[1]);
          printf("\tY-Axis Map (Sign): %d (%d)\r\n", imu_axis[2], imu_axis[3]);
          printf("\tZ-Axis Map (Sign): %d (%d)\r\n", imu_axis[4], imu_axis[5]);
     }
     return err;
}

/*
 ██████  ███████ ████████         ██     ███████ ███████ ████████
██       ██         ██           ██      ██      ██         ██
██   ███ █████      ██          ██       ███████ █████      ██
██    ██ ██         ██         ██             ██ ██         ██
 ██████  ███████    ██        ██         ███████ ███████    ██
*/

/** Class Getters */
float BNO055_I2C::get_accel_conversion_factor(){ return this->accel_conversion_factor; }
float BNO055_I2C::get_mag_conversion_factor(){ return this->mag_conversion_factor; }
float BNO055_I2C::get_gyro_conversion_factor(){ return this->gyro_conversion_factor; }

/** Class Setters */
void BNO055_I2C::set_accel_conversion_factor(float factor){ this->accel_conversion_factor = factor; }
void BNO055_I2C::set_mag_conversion_factor(float factor){ this->mag_conversion_factor = factor; }
void BNO055_I2C::set_gyro_conversion_factor(float factor){ this->gyro_conversion_factor = factor; }

/*
███    ███ ██    ██ ██   ██     ███████ ██████  ███████  ██████ ██ ███████ ██  ██████
████  ████ ██    ██  ██ ██      ██      ██   ██ ██      ██      ██ ██      ██ ██
██ ████ ██ ██    ██   ███       ███████ ██████  █████   ██      ██ █████   ██ ██
██  ██  ██ ██    ██  ██ ██           ██ ██      ██      ██      ██ ██      ██ ██
██      ██  ██████  ██   ██     ███████ ██      ███████  ██████ ██ ██      ██  ██████
*/

uint8_t BNO055_I2C::get_my_channel(){ return this->_mux_channel; }
void BNO055_I2C::set_my_channel(uint8_t channel){ this->_mux_channel = channel; }

int BNO055_I2C::get_mux_channel(){
     if(this->_mux) return this->_mux->get_selected_channel();
     else return -1;
}

void BNO055_I2C::set_mux_channel(uint8_t channel){
     if(this->_mux) this->_mux->select_channel(channel);
}

void BNO055_I2C::attach_i2c_mux(TCA9548A* mux){
     if(!this->_mux) this->_mux = mux;
     else printf("[INFO] BNO055_I2C::attach_i2c_mux() ---- i2c multiplexer already attached.\r\n");
}

void BNO055_I2C::detach_i2c_mux(){
     if(this->_mux) this->_mux = nullptr;
     else printf("[INFO] BNO055_I2C::detach_i2c_mux() ---- No multiplexer to detach.\r\n");
}

bool BNO055_I2C::isMuxPresent(bool verbose){
     if(this->_mux){
          if(verbose) printf("[INFO] BNO055_I2C::isMuxPresent() ---- Already has an attached multiplexer.\r\n");
          return true;
     } else{
          if(verbose) printf("[INFO] BNO055_I2C::isMuxPresent() ---- Has no multiplexer.\r\n");
          return false;
     }
}

bool BNO055_I2C::isMuxChannelSelected(bool verbose, bool debug){
     if(this->_mux){
          int curCh = this->_mux->get_selected_channel();
          if(debug) printf("[INFO] BNO055_I2C::isMuxChannelSelected() --- Multiplexer is currently on channel %d\r\n", curCh);
          if(curCh != this->_mux_channel){
               if(verbose) printf("[INFO] BNO055_I2C::isMuxChannelSelected() --- Switching to channel %d\r\n", this->_mux_channel);
               this->_mux->select_channel(this->_mux_channel,verbose, debug);
               return false;
          }
     }else{ if(verbose) printf("[WARNING] BNO055_I2C::isMuxChannelSelected() ---- Has no multiplexer.\r\n");}
     return true;
}


/*
██ ███    ███ ██    ██     ███████ ██████  ███████  ██████ ██ ███████ ██  ██████
██ ████  ████ ██    ██     ██      ██   ██ ██      ██      ██ ██      ██ ██
██ ██ ████ ██ ██    ██     ███████ ██████  █████   ██      ██ █████   ██ ██
██ ██  ██  ██ ██    ██          ██ ██      ██      ██      ██ ██      ██ ██
██ ██      ██  ██████      ███████ ██      ███████  ██████ ██ ██      ██  ██████
*/

int BNO055_I2C::set_mode(uint8_t mode){
     bool isReady = this->isMuxChannelSelected();
     int err = this->write_byte(BNO055_OPR_MODE_ADDR, mode & 0xFF);
     usleep(0.03 * 1000000);
     return err;
}

void BNO055_I2C::_config_mode(){this->set_mode(OPERATION_MODE_CONFIG);}
void BNO055_I2C::_operation_mode(){this->set_mode(this->_mode);}

/** ==================================================================
*					IMU-Specific SETTERS
* ==================================================================== */

void BNO055_I2C::set_external_crystal(bool use_external_crystal){
     bool isReady = this->isMuxChannelSelected();
     // Switch to configuration mode.
     this->_config_mode();
     if(use_external_crystal) this->write_byte(BNO055_SYS_TRIGGER_ADDR, 0x80);
     else this->write_byte(BNO055_SYS_TRIGGER_ADDR, 0x00);
     // Go back to normal operation mode.
     this->_operation_mode();
}

int BNO055_I2C::set_calibration(uint8_t* data){
     bool isReady = this->isMuxChannelSelected();
     int numBytes = (sizeof(data)/sizeof(*data));
     printf("[INFO] BNO055_I2C::set_calibration() --- Length of input buffer = %d\r\n",numBytes);
     if(numBytes != 22){
          printf("[ERROR] BNO055_I2C::set_calibration() --- Expecting input data to be 22 bytes.\r\n");
          return -1;
     }
     this->_config_mode();
     int err = this->write_bytes(ACCEL_OFFSET_X_LSB_ADDR, (char*)data, 22);
     this->_operation_mode();
     return err;
}

int BNO055_I2C::set_axis_remap(int x, int y, int z, int xsign, int ysign, int zsign){
     bool isReady = this->isMuxChannelSelected();
     this->_config_mode();

     // Set the axis remap register value.
     uint8_t map_config = 0x00;
     map_config = map_config | ((z & 0x03) << 4);
     map_config = map_config | ((y & 0x03) << 2);
     map_config = map_config | (x & 0x03);
     this->write_byte(BNO055_AXIS_MAP_CONFIG_ADDR, map_config);
     // Set the axis remap sign register value.
     uint8_t sign_config = 0x00;
     sign_config = sign_config | ((xsign & 0x01) << 2);
     sign_config = sign_config | ((ysign & 0x01) << 1);
     sign_config = sign_config | (zsign & 0x01);
     this->write_byte(BNO055_AXIS_MAP_SIGN_ADDR, sign_config);

     this->_operation_mode();
     return 0;
}

/** ==================================================================
*					IMU-Specific GETTERS
* ==================================================================== */

void BNO055_I2C::get_revision(int* revision_data){
     bool isReady = this->isMuxChannelSelected();
     // Read registers
     uint8_t sw_msb = this->read_raw_byte(BNO055_SW_REV_ID_MSB_ADDR);
     uint8_t sw_lsb = this->read_raw_byte(BNO055_SW_REV_ID_LSB_ADDR);
     uint8_t blId = this->read_raw_byte(BNO055_BL_REV_ID_ADDR);
     uint8_t accelId = this->read_raw_byte(BNO055_ACCEL_REV_ID_ADDR);
     uint8_t gyroId = this->read_raw_byte(BNO055_GYRO_REV_ID_ADDR);
     uint8_t magId = this->read_raw_byte(BNO055_MAG_REV_ID_ADDR);

     uint16_t swId = ((sw_msb << 8) | sw_lsb) & 0xFFFF;

     int tmp[5] = {(int)swId, (int)blId, (int)accelId, (int)gyroId, (int)magId};
     memcpy(revision_data,tmp,5*sizeof(int));
}

void BNO055_I2C::get_system_status(int* status, bool run_self_test){
     bool isReady = this->isMuxChannelSelected();
     uint8_t self_test = -1;
     if(run_self_test){
          // Switch to configuration mode if running self test.
          this->_config_mode();
          // Perform a self test.
          uint8_t sys_trigger = this->read_raw_byte(BNO055_SYS_TRIGGER_ADDR);
          this->write_byte(BNO055_SYS_TRIGGER_ADDR, sys_trigger | 0x1);
          // Wait for self test to finish.
          usleep(1000000);
          // Read test result.
          self_test = this->read_raw_byte(BNO055_SELFTEST_RESULT_ADDR);
          // Go back to operation mode.
          this->_operation_mode();
     }
     // Now read status and error registers.
     uint8_t _status = this->read_raw_byte(BNO055_SYS_STAT_ADDR);
     uint8_t error = this->read_raw_byte(BNO055_SYS_ERR_ADDR);

     int tmp[3] = {(int)_status, (int)self_test, (int)error};
     memcpy(status,tmp,3*sizeof(int));
}

void BNO055_I2C::get_calibration_status(int* status){
     bool isReady = this->isMuxChannelSelected();
     uint8_t calibStatus = this->read_raw_byte(BNO055_CALIB_STAT_ADDR);

     uint8_t sysStat = (calibStatus >> 6) & 0x03;
     uint8_t gyroStat = (calibStatus >> 4) & 0x03;
     uint8_t accelStat = (calibStatus >> 2) & 0x03;
     uint8_t magStat = calibStatus & 0x03;

     int tmp[4] = {(int)sysStat, (int)gyroStat, (int)accelStat, (int)magStat};
     memcpy(status,tmp,4*sizeof(int));
}

// void BNO055_I2C::get_calibration(float* data){
void BNO055_I2C::get_calibration(int* data){
     bool isReady = this->isMuxChannelSelected();
     // float tmp[11];
     int tmp[11];
     int16_t _data[11];

     this->_config_mode();
     int nRead = this->read_vector(ACCEL_OFFSET_X_LSB_ADDR, &_data[0],11);
     this->_operation_mode();
     // printf("[INFO] BNO055_I2C::get_calibration() ---- %d bytes read from register 0x%02X\r\n\t",nRead,_address);
     // tmp[0] = (float) _data[0] / this->accel_conversion_factor;
     // tmp[1] = (float) _data[1] / this->accel_conversion_factor;
     // tmp[2] = (float) _data[2] / this->accel_conversion_factor;
     // tmp[3] = (float) _data[3] / this->mag_conversion_factor;
     // tmp[4] = (float) _data[4] / this->mag_conversion_factor;
     // tmp[5] = (float) _data[5] / this->mag_conversion_factor;
     // tmp[6] = (float) _data[6] / this->gyro_conversion_factor;
     // tmp[7] = (float) _data[7] / this->gyro_conversion_factor;
     // tmp[8] = (float) _data[8] / this->gyro_conversion_factor;
     // tmp[9] = (float) _data[9];
     // tmp[10] = (float) _data[10];

     tmp[0] = (int) _data[0]; // / this->accel_conversion_factor;
     tmp[1] = (int) _data[1]; // / this->accel_conversion_factor;
     tmp[2] = (int) _data[2]; // / this->accel_conversion_factor;
     tmp[3] = (int) _data[3]; // / this->mag_conversion_factor;
     tmp[4] = (int) _data[4]; // / this->mag_conversion_factor;
     tmp[5] = (int) _data[5]; // / this->mag_conversion_factor;
     tmp[6] = (int) _data[6]; // / this->gyro_conversion_factor;
     tmp[7] = (int) _data[7]; // / this->gyro_conversion_factor;
     tmp[8] = (int) _data[8]; // / this->gyro_conversion_factor;
     tmp[9] = (int) _data[9];
     tmp[10] = (int) _data[10];

     memcpy(data,tmp, sizeof(tmp));
}

void BNO055_I2C::get_axis_remap(int* data){
     bool isReady = this->isMuxChannelSelected();
     // Get the axis remap register value.
     uint8_t map_config = this->read_raw_byte(BNO055_AXIS_MAP_CONFIG_ADDR);
     uint8_t z = (map_config >> 4) & 0x03;
     uint8_t y = (map_config >> 2) & 0x03;
     uint8_t x = map_config & 0x03;

     // Get the axis remap sign register value.
     uint8_t sign_config = this->read_raw_byte(BNO055_AXIS_MAP_SIGN_ADDR);
     uint8_t x_sign = (sign_config >> 2) & 0x01;
     uint8_t y_sign = (sign_config >> 1) & 0x01;
     uint8_t z_sign = sign_config & 0x01;

     int tmp[6] = {(int)x, (int)x_sign, (int)y, (int)y_sign, (int)z, (int)z_sign};
     memcpy(data,tmp, sizeof(tmp));
}

int BNO055_I2C::read_vector(uint8_t _address, int16_t* data, int count){
     bool isReady = this->isMuxChannelSelected();
     int nBytes = count*2;
     char buf[nBytes];
     uint16_t byte_array[count];

     // int nRead = this->_read_bytes(_address, nBytes, (uint8_t*)&buf[0]);
     int nRead = this->read_bytes(_address, &buf[0],nBytes);
     // printf("[INFO] BNO055_I2C::read_vector() ---- %d bytes read from register 0x%02X\r\n",nRead,_address);
     for(int i = 0; i < count; i++){
          uint8_t byte_msb = (uint8_t)buf[i*2+1];
          uint8_t byte_lsb = (uint8_t)buf[i*2];
          uint16_t tmp_byte = byte_msb << 8 | (byte_lsb & 0xFF);

          int16_t new_byte;
          if(tmp_byte > 32767) new_byte = tmp_byte - 65536;
          else  new_byte = (int16_t) tmp_byte;

          byte_array[i] = (int16_t) new_byte;
     }
     memcpy(data,byte_array, sizeof(int16_t) * count);
     if(nRead >= 0) return nRead;
     else return 1;
}

void BNO055_I2C::get_euler(float* data, bool verbose){
     int16_t _data[3];
     float tmp[3];
     this->read_vector(BNO055_EULER_H_LSB_ADDR, &_data[0]);
     /** Re-arrange reutrned array as R, P, Y b/c BNO-055 returns Y, R, P */
     tmp[2] = (float) _data[0] / 16.0;
     tmp[0] = (float) _data[1] / 16.0;
     tmp[1] = (float) _data[2] / 16.0;
     if(verbose) printf("Euler Angles: %f, %f, %f\r\n", tmp[0], tmp[1] , tmp[2]);
     memcpy(data,tmp, sizeof(tmp));
}

void BNO055_I2C::get_magnetometer(float* data, bool verbose){
     int16_t _data[3];
     float tmp[3];
     this->read_vector(BNO055_MAG_DATA_X_LSB_ADDR, &_data[0]);
     tmp[0] = (float) _data[0] / 16.0;
     tmp[1] = (float) _data[1] / 16.0;
     tmp[2] = (float) _data[2] / 16.0;
     if(verbose) printf("Magnetometer (micro-Teslas) X, Y, Z: %f, %f, %f\r\n", tmp[0], tmp[1] , tmp[2]);
     memcpy(data,tmp, sizeof(tmp));
}

void BNO055_I2C::get_gyroscope(float* data, bool verbose){
     int16_t _data[3];
     float tmp[3];
     this->read_vector(BNO055_GYRO_DATA_X_LSB_ADDR, &_data[0]);
     tmp[0] = (float) _data[0] / 900.0;
     tmp[1] = (float) _data[1] / 900.0;
     tmp[2] = (float) _data[2] / 900.0;
     if(verbose) printf("Gyroscope (rad/sec) X, Y, Z: %f, %f, %f\r\n", tmp[0], tmp[1] , tmp[2]);
     memcpy(data,tmp, sizeof(tmp));
}

void BNO055_I2C::get_accelerometer(float* data, bool verbose){
     int16_t _data[3];
     float tmp[3];
     this->read_vector(BNO055_ACCEL_DATA_X_LSB_ADDR, &_data[0]);
     tmp[0] = (float) _data[0] / 100.0;
     tmp[1] = (float) _data[1] / 100.0;
     tmp[2] = (float) _data[2] / 100.0;
     if(verbose) printf("Accelerometer (m/s^2) X, Y, Z: %f, %f, %f\r\n", tmp[0], tmp[1] , tmp[2]);
     memcpy(data,tmp, sizeof(tmp));
}

void BNO055_I2C::get_linear_acceleration(float* data, bool verbose){
     int16_t _data[3];
     float tmp[3];
     this->read_vector(BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, &_data[0]);
     tmp[0] = (float) _data[0] / 100.0;
     tmp[1] = (float) _data[1] / 100.0;
     tmp[2] = (float) _data[2] / 100.0;
     if(verbose) printf("Linear Acceleration (m/s^2) X, Y, Z: %f, %f, %f\r\n", tmp[0], tmp[1] , tmp[2]);
     memcpy(data,tmp, sizeof(tmp));
}

void BNO055_I2C::get_gravity(float* data, bool verbose){
     int16_t _data[3];
     float tmp[3];
     this->read_vector(BNO055_GRAVITY_DATA_X_LSB_ADDR, &_data[0]);
     tmp[0] = (float) _data[0] / 100.0;
     tmp[1] = (float) _data[1] / 100.0;
     tmp[2] = (float) _data[2] / 100.0;
     if(verbose) printf("Gravity (m/s^2) X, Y, Z: %f, %f, %f\r\n", tmp[0], tmp[1] , tmp[2]);
     memcpy(data,tmp, sizeof(tmp));
}

void BNO055_I2C::get_quaternions(float* data, bool verbose){
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

float BNO055_I2C::get_temperature(bool verbose){
     bool isReady = this->isMuxChannelSelected();
     int8_t data;
     uint8_t udata = this->read_raw_byte(BNO055_TEMP_ADDR);
     if(udata < 0){
          printf("[ERROR] BNO055_I2C::_read_signed_byte ---- 'BNO055_I2C::_read_byte' failed with error code [%d]\r\n", udata);
          return -1;
     }
     if(udata > 127) data = udata - 256;
     else data = udata;
     if(verbose) printf("Temperature [Celsius]: %f\r\n", (float)data);
     return (float)data;
}
