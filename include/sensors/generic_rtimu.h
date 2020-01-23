#ifndef GENERIC_RTIMU_H_
#define GENERIC_RTIMU_H_

#include <vector>
#include <RTIMULib.h>

using namespace std;

class GenericRTIMU{

private:
     // TODO: Figure out how to account for initial pose offsets to start everything off by 0

     string calib_path;
     string calib_file;

     int num_updates;
     float declination_offset;          // (radians)
     float corrected_yaw;

     RTIMU* _imu;
     RTIMUSettings* _settings;

public:
	GenericRTIMU();
	GenericRTIMU(string path, string file);
     ~GenericRTIMU();

     float accel[3];
     float gyro[3];
     float mag[3];
     float euler[3];
     float quats[4];
     uint32_t time_start;
     uint32_t now;

     // TODO: potentially integrate to get an estimated positional movement
     float position[3];

     int init(string path, string file);
     int update(float timeout);

     void print_settings(); // TODO: extract settings from calibration.ini file
     void print_data();
     void print_angles();

     int get_update_period();
     vector<float> get_raw_data();
     vector<float> get_all_data();

     // TODO: Add these functions for ease-of-use later
     // void reset_yaw();

};

#endif /* GENERIC_RTIMU_H_ */
