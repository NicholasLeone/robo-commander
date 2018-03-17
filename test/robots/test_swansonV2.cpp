#include <iostream>

#include <pigpiod_if2.h>
#include "swansonV2.h"

using namespace std;

int main(int argc, char *argv[]){

     float max_vel = 0.5;
     float max_omega = max_vel / 0.381;

     int pi = pigpio_start(NULL, NULL); /* Connect to Pi. */

     if (pi >= 0){

          SwansonV2 bot(pi);

          while(1){
               bot.readRC();
               float accel = (float) bot.controls.speed / 1000000;
               float omega = (float) bot.controls.yaw / 1000000;

               accel = max_vel * accel;
               omega = max_omega * omega;

               //cout << "Controls: " << accel << ",          " << omega << endl;
               bot.drive(accel, omega);
               bot.updateSensors();
          }

          pigpio_stop(pi);
     }

     return 0;
}
