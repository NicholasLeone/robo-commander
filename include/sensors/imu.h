#ifndef IMU_H_
#define IMU_H_

#include <vector>
#include <RTIMULib.h>

using namespace std;

class IMU{

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
	IMU();
	IMU(string path, string file);
     ~IMU();

     float accel[3];
     float gyro[3];
     float mag[3];
     float euler[3];
     float quats[4];

     // TODO: potentially integrate to get an estimated positional movement
     float position[3];

     int init(string path, string file);
     void update();
     int get_update_period();

     // TODO: Add these functions for ease-of-use later
     // void reset_yaw();

     // void print();

};

#endif /* IMU_H_*/
