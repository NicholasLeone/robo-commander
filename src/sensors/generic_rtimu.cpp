#include <string.h>
#include <iostream>
#include <unistd.h>
#include <chrono>

#include "base/definitions.h"
#include "sensors/generic_rtimu.h"

static const float G_TO_MPSS = 9.80665;
static const int uT_TO_T = 1000000;

using namespace std;
using namespace chrono;

GenericRTIMU::GenericRTIMU(){
     num_updates = 0;
     declination_offset = -0.6;
}

GenericRTIMU::GenericRTIMU(string path, string file){
     num_updates = 0;
     declination_offset = -0.6;

     int err = init(path, file);
     if(err < 0){
          exit(1);
     }
}

GenericRTIMU::~GenericRTIMU(){
     delete _imu;
     delete _settings;
}

int GenericRTIMU::init(string path, string file){
     calib_path = path;
     calib_file = file;

     RTIMUSettings* tmpSettings = new RTIMUSettings(path.c_str(), file.c_str());
     RTIMU* tmpImu = RTIMU::createIMU(tmpSettings);

     if( (tmpImu == NULL) || (tmpImu->IMUType() == RTIMU_TYPE_NULL)){
          printf("ERROR: No GenericRTIMU found, or could not be opened!\r\n");
          delete tmpSettings;
          delete tmpImu;
          return -1;
     }else{
          _settings = tmpSettings;
          _imu = tmpImu;
          printf("SUCCESS: GenericRTIMU opened, attempting to initialize...\r\n");
     }

     if(!_imu->IMUInit()){
          printf("ERROR: GenericRTIMU could not be initialized!\r\n");
          delete _settings;
          delete _imu;
          return -2;
     }

     // Set the Fusion coefficient
     _imu->setSlerpPower(0.02);
     // Enable the sensors
     _imu->setGyroEnable(true);
     _imu->setAccelEnable(true);
     _imu->setCompassEnable(true);

     // Get current system time for later
     time_start = RTMath::currentUSecsSinceEpoch();

     printf("SUCCESS: GenericRTIMU initialized!\r\n");
     return 1;
}

int GenericRTIMU::get_update_period(){ return _imu->IMUGetPollInterval() * 1000; }

int GenericRTIMU::update(float timeout){
     int err = -1;
     high_resolution_clock::time_point start = high_resolution_clock::now();
     // printf("[INFO] GenericRTIMU::update() --- Reading until valid data received.\r\n");
     while(1){
          high_resolution_clock::time_point now = high_resolution_clock::now();
          if(!this->_imu->IMURead()){
               duration<float> time_span = duration_cast<duration<float>>(now - start);
               float elapsed_time = time_span.count();
               if(elapsed_time >= timeout){
                    printf("[ERROR] GenericRTIMU::update() --- Unable to receive valid IMU data within %.3f seconds.\r\n", timeout);
                    err = -100;
                    break;
               }
          } else{
               err = 0;
               break;
          }
     }

     if(err >= 0){
          RTIMU_DATA data = _imu->getIMUData();
          this->num_updates++;
          this->now = RTMath::currentUSecsSinceEpoch();

          this->accel[0] = data.accel.x() * G_TO_MPSS;
          this->accel[1] = data.accel.y() * G_TO_MPSS;
          this->accel[2] = data.accel.z() * G_TO_MPSS;

          this->gyro[0] = data.gyro.x();
          this->gyro[1] = data.gyro.y();
          this->gyro[2] = data.gyro.z();

          this->quats[0] = data.fusionQPose.x();
		this->quats[1] = data.fusionQPose.y();
		this->quats[2] = data.fusionQPose.z();
		this->quats[3] = data.fusionQPose.scalar();

          // TODO: Potentially need to update timestamp for this section
		if(data.compassValid){
			this->mag[0] = data.compass.x() / uT_TO_T;
               this->mag[1] = data.compass.y() / uT_TO_T;
			this->mag[2] = data.compass.z() / uT_TO_T;
		}

          this->euler[0] = data.fusionPose.x();
          this->euler[1] = data.fusionPose.y();
          this->euler[2] = data.fusionPose.z();

          this->corrected_yaw = this->euler[2] - this->declination_offset;
	}
     return err;
}

vector<float> GenericRTIMU::get_raw_data(){
     vector<float> out;
     out.reserve(9);

     out.push_back(accel[0]);
     out.push_back(accel[1]);
     out.push_back(accel[2]);
     out.push_back(gyro[0]);
     out.push_back(gyro[1]);
     out.push_back(gyro[2]);
     out.push_back(mag[0]);
     out.push_back(mag[1]);
     out.push_back(mag[2]);

     return out;
}

vector<float> GenericRTIMU::get_all_data(){
     vector<float> out;
     out.reserve(16);

     out.push_back(accel[0]);
     out.push_back(accel[1]);
     out.push_back(accel[2]);
     out.push_back(gyro[0]);
     out.push_back(gyro[1]);
     out.push_back(gyro[2]);
     out.push_back(mag[0]);
     out.push_back(mag[1]);
     out.push_back(mag[2]);

     out.push_back(quats[0]);
     out.push_back(quats[1]);
     out.push_back(quats[2]);
     out.push_back(quats[3]);

     float roll = fmod((euler[0]*M_RAD2DEG + 360.0),360.0);
     float pitch = fmod((euler[1]*M_RAD2DEG + 360.0),360.0);
     float yaw = fmod((euler[2]*M_RAD2DEG + 360.0),360.0);

     out.push_back(roll);
     out.push_back(pitch);
     out.push_back(yaw);

     return out;
}

void GenericRTIMU::print_settings(){
     // Print out all configured parameters for debugging
     // printf("GenericRTIMU CONFIGURATION SETTINGS: \r\n");
     // printf("       Device Address: %s\r\n", _add);
     // printf("       Baud Rate: %d\r\n", _baud);
     // printf("       GenericRTIMU Type: %.4f\r\n", _type);
     // printf("\r\n");
}

void GenericRTIMU::print_data(){
     fflush(stdout);
     printf("GenericRTIMU DATA: \r\n");
     printf("       Accelerations (m/s^2): %.4f        %.4f      %.4f\r\n", accel[0], accel[1], accel[2]);
     printf("       Angular Velocities (rad/sec): %.4f        %.4f      %.4f\r\n", gyro[0], gyro[1], gyro[2]);
     printf("       Magnetometer (μT): %.4f        %.4f      %.4f\r\n", mag[0], mag[1], mag[2]);
     printf("       Fused Euler Angles (deg): %.4f        %.4f      %.4f\r\n\r\n", euler[0]*M_RAD2DEG, euler[1]*M_RAD2DEG, euler[2]*M_RAD2DEG);
}

void GenericRTIMU::print_angles(){
     float tmpAng[3];
     for(int i = 0; i<3;i++){ tmpAng[i] = euler[i] * M_RAD2DEG; }
     fflush(stdout);
     printf("       Fused Euler Angles (deg): %.4f        %.4f      %.4f\r\n\r\n", tmpAng[0], tmpAng[1], tmpAng[2]);
}
