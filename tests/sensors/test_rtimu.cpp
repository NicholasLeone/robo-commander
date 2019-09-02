#include <iostream>
#include "base/definitions.h"

#include "sensors/generic_rtimu.h"

using namespace std;

int main(int argc, char *argv[]){

	float angle;
	int dt;

	string path = "/home/hunter/devel/robo-dev/config/sensors";
	string file = "mpu9250";

     GenericRTIMU imu(path, file);

	while(1){
		imu.update();
		angle = fmod((imu.euler[1]*M_RAD2DEG + 360.0),360.0);
		dt = imu.get_update_period();
		usleep(dt);

		cout << "Angle: " << angle << endl;
	}

     return 0;
}
