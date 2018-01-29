#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <bitset>
#include <vector>
#include <pigpiod_if2.h>
#include "mpu9250.h"
#include "utils/utils.h"

// TODO: Magnetometer  #define REG_MAG

using namespace std;

MPU9250::MPU9250(INTERFACE_PARAMS in_iface, IMU_CONFIG in_config){

     // Temporary Storage of interface options for communication setup error checking
     int pi, com, err;
     int data[32];
     int reg[16];

     // Attempt Communicating with Sensor
     if(!in_iface.cMeth.compare("serial")){

          _ser = new SerialDev(in_iface.serParams.add, in_iface.serParams.baud);

          if(_ser->active)
               com = 1;
          else
               com = -1;

     }else{
          pi = pigpio_start(in_iface.host, in_iface.port);
          // Check if communication with Raspberry Pi Succeeded
          if(pi >= 0){ // Success
               // Attempt to communication to device
               if(!in_iface.cMeth.compare("i2c"))
                    com = i2c_open(pi, in_iface.i2c.bus, in_iface.i2c.add, 0);
               else
                    printf("INPUT ERROR: Communication method to device not specified. \r\n");

          } else
               printf("SETUP ERROR: Failed to connect to Raspberry Pi. (Error Code = %d)\r\n", pi);
     }

     // Check for communication setup failure
     if(com < 0)
          printf("SETUP ERROR: Failed to establish communications with device. (Error Code = %d)\r\n", com);

     // Store created handles in struct for later usage
     in_iface.pi = pi;
     in_iface.com = com;

     this->iface = in_iface;
     _pi = pi;
     _com = com;

     this->accel.add = REG_ACCEL_DATA;
     this->accel.fs = in_config.fs_accel;
     this->gyro.add = REG_GYRO_DATA;
     this->gyro.fs = in_config.fs_gyro;
     this->mag.add = REG_MAG_ADD;
     this->mag.fs = in_config.fs_mag;

     vector<float> tmpEuler = getEuler();
     this->att.roll = tmpEuler.at(0);
     this->att.pitch = tmpEuler.at(1);
     this->att.yaw = tmpEuler.at(2);


#ifdef TEST_CALIBRATION
     printf("MPU-9250: Initialized\r\nCalibrating...\r\n");
     this->calibrateAccel();
     sleep(1);
#endif
}

MPU9250::~MPU9250(){

     i2c_close(_pi, _com);
     // _ser;
     pigpio_stop(_pi);

}

vector<float> MPU9250::updateSerial(){

     string delimiter = ", ";
     string tmpStr = _ser->readLine();
     vector<float> data =  parseFloat(tmpStr, delimiter);

     return data;
}

void MPU9250::updateAccel(){

     uint8_t buf[6];
     int data[3];
     float tmpX, tmpY, tmpZ;

     if(!iface.cMeth.compare("serial")){
          vector<float> serData = updateSerial();

          tmpX = serData.at(1);
          tmpY = serData.at(2);
          tmpZ = serData.at(3);

     }else if(!iface.cMeth.compare("i2c")){
          int returnCount = i2c_read_i2c_block_data(_pi, _com, REG_ACCEL_DATA, (char*)buf,6);
          float sensitivity = this->accel.sens;

          data[0] = (int16_t)((((uint16_t) buf[0]) << 8) | (uint8_t) buf[1]);
     	data[1] = (int16_t)((((uint16_t) buf[2]) << 8) | (uint8_t) buf[3]);
     	data[2] = (int16_t)((((uint16_t) buf[4]) << 8) | (uint8_t) buf[5]);
     	tmpX =  (float) data[0] / sensitivity;
     	tmpY =  (float) data[1] / sensitivity;
     	tmpZ =  (float) data[2] / sensitivity;
     }

#ifdef USE_OFFSETS
     tmpX = tmpX - accel.x_off;
     tmpY = tmpY - accel.y_off;
     tmpZ = tmpZ - accel.z_off;
#endif
	printf("Accel: %.2f   %.2f   %.2f  \r\n",tmpX, tmpY, tmpZ);

     this->accel.x = tmpX;
     this->accel.y = tmpY;
     this->accel.z = tmpZ;
}



void MPU9250::updateGyro(){

     uint8_t buf[6];
     int data[3];
     float tmpX, tmpY, tmpZ;

     if(!iface.cMeth.compare("serial")){
          vector<float> serData = updateSerial();

          tmpX = serData.at(4);
          tmpY = serData.at(5);
          tmpZ = serData.at(6);

     }else if(!iface.cMeth.compare("i2c")){
          int returnCount = i2c_read_i2c_block_data(_pi, _com, REG_GYRO_DATA, (char*)buf,6);
          float sensitivity = this->gyro.sens;

          data[0] = (int16_t)((((uint16_t) buf[0]) << 8) | (uint8_t) buf[1]);
     	data[1] = (int16_t)((((uint16_t) buf[2]) << 8) | (uint8_t) buf[3]);
     	data[2] = (int16_t)((((uint16_t) buf[4]) << 8) | (uint8_t) buf[5]);
     	tmpX =  (float) data[0] / sensitivity;
     	tmpY =  (float) data[1] / sensitivity;
     	tmpZ =  (float) data[2] / sensitivity;
     }

#ifdef USE_OFFSETS
     tmpX = tmpX - gyro.x_off;
     tmpY = tmpY - gyro.y_off;
     tmpZ = tmpZ - gyro.z_off;
#endif

	printf("Gyro: %.3f   %.3f   %.3f  \r\n",tmpX, tmpY, tmpZ);

     this->gyro.x = tmpX;
     this->gyro.y = tmpY;
     this->gyro.z = tmpZ;
}

//TODO: Handle i2c
void MPU9250::updateMag(){

     uint8_t buf[6];
     int data[3];
     float tmpX, tmpY, tmpZ;

     if(!iface.cMeth.compare("serial")){
          vector<float> serData = updateSerial();

          tmpX = serData.at(7);
          tmpY = serData.at(8);
          tmpZ = serData.at(9);

     }else if(!iface.cMeth.compare("i2c")){
          int returnCount = i2c_read_i2c_block_data(_pi, _com, REG_GYRO_DATA, (char*)buf,6);
          float sensitivity = this->gyro.sens;

          data[0] = (int16_t)((((uint16_t) buf[0]) << 8) | (uint8_t) buf[1]);
     	data[1] = (int16_t)((((uint16_t) buf[2]) << 8) | (uint8_t) buf[3]);
     	data[2] = (int16_t)((((uint16_t) buf[4]) << 8) | (uint8_t) buf[5]);
     	tmpX =  (float) data[0] / sensitivity;
     	tmpY =  (float) data[1] / sensitivity;
     	tmpZ =  (float) data[2] / sensitivity;
     }

#ifdef USE_OFFSETS
     tmpX = tmpX - mag.x_off;
     tmpY = tmpY - mag.y_off;
     tmpZ = tmpZ - mag.z_off;
#endif

	printf("Mag: %.3f   %.3f   %.3f  \r\n",tmpX, tmpY, tmpZ);

     this->mag.x = tmpX;
     this->mag.y = tmpY;
     this->mag.z = tmpZ;


}



/** REFERENCE FOR LATER
void calibrateSensors() {
  for (uint8_t i = 0; i < 100; i++) { // Take the average of 100 readings
    zeroValues[0] += analogRead(gyroY);
    zeroValues[1] += analogRead(accY);
    zeroValues[2] += analogRead(accZ);
    delay(10);
  }
  zeroValues[0] /= 100; // Gyro X-axis
  zeroValues[1] /= 100; // Accelerometer Y-axis
  zeroValues[2] /= 100; // Accelerometer Z-axis

  if(zeroValues[1] > 500) { // Check which side is lying down - 1g is equal to 0.33V or 102.3 quids (0.33/3.3*1023=102.3)
    zeroValues[1] -= 102.3; // +1g when lying at one of the sides
    kalman.setAngle(90); // It starts at 90 degress and 270 when facing the other way
    gyroAngle = 90;
  } else {
    zeroValues[1] += 102.3; // -1g when lying at the other side
    kalman.setAngle(270);
    gyroAngle = 270;
  }

  digitalWrite(buzzer,HIGH);
  delay(100);
  digitalWrite(buzzer,LOW);
}
*/

// TODO: i2c handling
void MPU9250::calibrateAccel(){
     float tmpX, tmpY, tmpZ;
     int count = 0;
     float sumx = 0;
     float sumy = 0;
     float sumz = 0;

     for(int i = 0;i<=MAX_CAL_STEPS-1;i++){
          if(!iface.cMeth.compare("serial")){
               vector<float> serData = updateSerial();

               sumx += serData.at(1);
               sumy += serData.at(2);
               sumz += serData.at(3);

          }else if(!iface.cMeth.compare("i2c")){

          }
          count++;
     }

     tmpX = sumx / float (count);
     tmpY = sumy / float (count);
     tmpZ = sumz / float (count);

     printf("Accel Offset: %.3f   %.3f   %.3f  \r\n",tmpX, tmpY, tmpZ);

     this->accel.x_off = tmpX;
     this->accel.y_off = tmpY;
     this->accel.z_off = tmpZ;

}

// TODO: i2c handling
void MPU9250::calibrateGyro(){
     float tmpX, tmpY, tmpZ;
     int count = 0;
     float sumx = 0;
     float sumy = 0;
     float sumz = 0;

     for(int i = 0;i<=MAX_CAL_STEPS-1;i++){
          if(!iface.cMeth.compare("serial")){
               vector<float> serData = updateSerial();

               sumx += serData.at(4);
               sumy += serData.at(5);
               sumz += serData.at(6);

          }else if(!iface.cMeth.compare("i2c")){

          }
          count++;
     }

     tmpX = sumx / float (count);
     tmpY = sumy / float (count);
     tmpZ = sumz / float (count);

     printf("Gyro Offset: %.3f   %.3f   %.3f  \r\n",tmpX, tmpY, tmpZ);

     this->gyro.x_off = tmpX;
     this->gyro.y_off = tmpY;
     this->gyro.z_off = tmpZ;

}


float MPU9250::getGyroSens(){

     int fs = this->gyro.fs;
     float sens;

     if(fs == 250)
          sens = 131.0;
     else if(fs == 500)
          sens = 65.5;
     else if(fs == 1000)
          sens = 32.8;
     else if(fs == 2000)
          sens = 16.4;
     else
          printf("ERROR: Gyro Full-Scale not set.\r\n");

     // Store Sensitivity in Sensor parameters
     this->gyro.sens = sens;

     return sens;
}

float MPU9250::getAccelSens(){

     int fs = this->accel.fs;
     float sens;

     if(fs == 2)
          sens = 16384.0;
     else if(fs == 4)
          sens = 8192.0;
     else if(fs == 8)
          sens = 4096.0;
     else if(fs == 16)
          sens = 2048.0;
     else
          printf("ERROR: Accel Full-Scale not set.\r\n");

     // Store Sensitivity in Sensor parameters
     this->accel.sens = sens;

     return sens;
}

float MPU9250::getMagSens(){

     int fs = this->mag.fs;
     float sens;

     if(fs >= 0)
          sens = 6.665;
     else
          printf("ERROR: Magnetometer Full-Scale not set.\r\n");

     // Store Sensitivity in Sensor parameters
     this->mag.sens = sens;

     return sens;
}


int MPU9250::setConfig(IMU_CONFIG config){

     int err, fs_g, fs_a,fs_m, val;
     int err_fs[3];
     int reg[2];
     int data[2];

     reg[0] = REG_GYRO_CONFIG;
     reg[1] = REG_ACCEL_CONFIG;

     fs_g = config.fs_gyro;
     fs_a = config.fs_accel;
     fs_m = config.fs_mag;

     // Gyroscope
     if(fs_g == 250)
          data[0] = 0b00000000;
     else if(fs_g == 500)
          data[0] = 0b00001000;
     else if(fs_g == 1000)
          data[0] = 0b00010000;
     else if(fs_g == 2000)
          data[0] = 0b00011000;
     else{
          printf("ERROR: Gyro Full-Scale not set.\r\n");
          return -1;
     }

     // Accelerometer
     if(fs_a == 2)
          data[1] = 0b00000000;
     else if(fs_a == 4)
          data[1] = 0b00001000;
     else if(fs_a == 8)
          data[1] = 0b00010000;
     else if(fs_a == 16)
          data[1] = 0b00011000;
     else{
          printf("ERROR: Accel Full-Scale not set.\r\n");
          return -2;
     }



     // Attempt to set IMU Full-Scale Ranges
     for(int i=0;i<=1;i++){
          err = i2c_write_byte_data(_pi, _com, reg[i], data[i]);
          if(err < 0)
               printf("I2C ERROR: I2C Write Failed. (Error Code = %d)\r\n", err);
          err_fs[i] = err;
     }

     return err;

}

IMU_CONFIG MPU9250::getConfig(){

     IMU_CONFIG tmpConfig;

     int buf[4];         // # of bytes associated to imu's config's
     int config[14];     // # of configuration options

	int err = i2c_read_i2c_block_data(_pi, _com, REG_CONFIG, (char*) buf, 4);

	config[0] = extract_bits(buf[0],6,6);	// FIFO MODE
	config[1] = extract_bits(buf[0],5,3);	// EXT_SYNC_SET
	config[2] = extract_bits(buf[0],2,0);	// DLPF_CFG

	config[3] = extract_bits(buf[1],7,7);	// XGYRO_Cten
	config[4] = extract_bits(buf[1],6,6);	// YGYRO_Cten
	config[5] = extract_bits(buf[1],5,5);	// ZGYRO_Cten
	config[6] = extract_bits(buf[1],4,3);	// GYRO_FS_SEL
	config[7] = extract_bits(buf[1],1,0);	// Fchoice_b

	config[8] = extract_bits(buf[2],7,7);	// ax_st_en
	config[9] = extract_bits(buf[2],6,6);	// ay_st_en
	config[10] = extract_bits(buf[2],5,5);	// az_st_en
	config[11] = extract_bits(buf[2],4,3);	// ACCEL_FS_SEL
	config[12] = extract_bits(buf[3],3,3);	// accel_fhoice_b
	config[13] = extract_bits(buf[3],2,0);	// A_DLPFCFG

#ifdef DEBUG_INTERMEDIATE_IMU
     if(err >= 0){
          cout << "Extracted Bytes: " << err << endl;

     	for(int i=0;i<=13;i++){
     		if(i<=2)
     			cout << "Config: ";
     		else if((i>2) && (i<=7))
     			cout << "Gyro: ";
     		else if(i>7)
     			cout << "Accel: ";

     		cout << config[i] << endl;
     	}
     } else
          printf("I2C Read Failed: %d \r\n", err);
#endif

	return tmpConfig;

}


float MPU9250::getEulerX(){

     vector<float> serData = updateSerial();
     float tmpAngle = serData.at(10);

}

float MPU9250::getEulerY(){

     vector<float> serData = updateSerial();
     float tmpAngle = serData.at(11);
}

float MPU9250::getEulerZ(){

     vector<float> serData = updateSerial();
     float tmpAngle = serData.at(12);

}

vector<float> MPU9250::getEuler(){

     vector<float> store(3);

     vector<float> serData = updateSerial();
     store.at(0) = serData.at(10);
     store.at(1) = serData.at(11);
     store.at(2) = serData.at(12);

     printf("Euler Angles [X, Y, Z]: %.2f	%.2f	%.2f\r\n", store.at(0),store.at(1),store.at(2));

     return store;
}
