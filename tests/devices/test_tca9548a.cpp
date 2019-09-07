#include <iostream>
#include <pigpiod_if2.h>
#include "devices/tca9548a.h"
#include "sensors/bno055_i2c.h"

using namespace std;

int main(int argc, char *argv[]){
	cout << "[START] Testing TCA-9548a Demo." << endl;
     int err;
     int channel;
	cout << "[START] PigpioD..." << endl;
     int pi = pigpio_start(NULL,NULL);
	cout << "[START] TCA-9548a..." << endl;
     TCA9548A mux(pi, 1);

	cout << "[START] Dummy i2c device..." << endl;
	// PCA9685 dummy(pi, 1, 0x28);
	BNO055_I2C dummy(pi, 1, 0x28);

     mux.scan_bus(&dummy,true);

     return 0;
}
