#include <iostream>
#include "base/definitions.h"

#include "sensors/generic_rtimu.h"

using namespace std;

int main(int argc, char *argv[]){
	string path = "/home/pi/devel/robo-commander/config/sensors";
	string file = "razor_m0_imu";

     GenericRTIMU imu(path, file);

	while(1){
		int err = imu.update(5.0);
		if(err >= 0){
			float angle = fmod((imu.euler[1]*M_RAD2DEG + 360.0),360.0);
			int dt = imu.get_update_period();
			usleep(dt);
			cout << "Angle: " << angle << endl;
		} else if(err <= -100){
			printf("[ERROR] TestGenericRTIMU::update() --- Unable to receive valid IMU data within specified timeout. Exiting...\r\n");
			break;
		}
	}

     return 0;
}
