#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <thread>

#include <pigpiod_if2.h>
#include "../../include/Utilities/utils.h"
#include "four_wd.h"

using namespace std;

int main(int argc, char *argv[]){

     int pi = pigpio_start(NULL, NULL); /* Connect to Pi. */

     if (pi >= 0){

          FourWD bot(pi);
          // thread upper(bot.updateSensors,&bot);

          while(1){
               bot.readRC();
               float accel = (float) bot.controls.speed / 1000000;
               float omega = (float) bot.controls.yaw / 1000000;
               bot.updateSensors();
               bot.drive(accel,-2 * omega);
          }
          // bot.drive(1,0);

          pigpio_stop(pi);
     }

     return 0;
}


/** TO COMPILE:

     g++ test_speed.cpp  -o TestSpeed -lpigpiod_if2 -Wall -pthread

*/
