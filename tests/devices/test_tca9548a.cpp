#include <iostream>
#include <pigpiod_if2.h>
#include "devices/tca9548a.h"
#include "devices/pca9685.h"

using namespace std;

int main(int argc, char *argv[]){
	cout << "[START] Testing PCA-9685 Demo." << endl;
     int err;
     int channel;
	cout << "[START] PigpioD..." << endl;
     int pi = pigpio_start(NULL,NULL);
	cout << "[START] TCA-9685..." << endl;
     TCA9548A mux(pi, 1);

	cout << "[START] Dummy i2c device..." << endl;
	PCA9685 pwm(pi, 1, 0x28);

     mux.scan_bus(&pwm,true);

     return 0;
}
