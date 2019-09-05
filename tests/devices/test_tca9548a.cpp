#include <iostream>
#include <pigpiod_if2.h>
#include "devices/tca9548a.h"

using namespace std;

int main(int argc, char *argv[]){
	cout << "[START] Testing PCA-9685 Demo." << endl;
     int err;
     int channel;
	cout << "[START] PigpioD..." << endl;
     int pi = pigpio_start(NULL,NULL);
	cout << "[START] TCA-9685..." << endl;
     TCA9548A mux(pi, 1);

     mux.scan_bus(0x28,true);

     return 0;
}
