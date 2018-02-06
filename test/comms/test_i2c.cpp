#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <math.h>

#include <pigpiod_if2.h>
#include "comms/i2c.h"
#include "base/definitions.h"

using namespace std;

int main(int argc, char *argv[]){
     int err;
     int bus = 1;
     int add = 0x70;
     int freq = 50;

     int on = 0;
     int off = 0;

     int pi = pigpio_start(NULL,NULL);
     I2C i2cDev(pi, bus, add);

     return 0;
}
