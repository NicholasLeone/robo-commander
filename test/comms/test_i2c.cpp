#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <pigpiod_if2.h>
#include "comms/i2c.h"

using namespace std;

int main(int argc, char *argv[]){
     int err;
     int bus = 1;
     int add = 0x70;

     int pi = pigpio_start(NULL,NULL);
     I2C i2cDev(pi, bus, add);

     return 0;
}
