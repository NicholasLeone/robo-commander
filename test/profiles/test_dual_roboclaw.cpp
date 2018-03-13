#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <thread>

#include <pigpiod_if2.h>
#include "dual_roboclaw.h"

using namespace std;

int main(int argc, char *argv[]){

     vector<int32_t> cmds;

     int pi = pigpio_start(NULL, NULL); /* Connect to Pi. */

     if (pi >= 0){

          DualClaw claws(pi);

          while(1){
               claws.update_status();
               claws.update_encoders();
               // cmds = claws.set_speeds(1.0, 0.0);
               // claws.drive(cmds);
          }

          pigpio_stop(pi);
     }

     return 0;
}
