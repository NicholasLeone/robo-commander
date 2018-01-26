#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <thread>
#include <pigpiod_if2.h>
#include "encoder.h"

using namespace std;

/** NOTE:   pigpiod_if2.h
*     This Library was made with the ability in mind to control multiple Pi's
* at once. Therefore, it is necessary to declare the host and port of the Pi you
* are working with.
*/
char *optHost = NULL; // Use NULL when working on localhost
char *optPort = NULL; // Use NULL when working on localhost
int optGpioA = 18;
int optGpioB = 23;
int optGlitch = 50;
int optSeconds = 60;
int optMode = ENCODER_MODE_STEP;

void callbackFun(int pos){
     // float spd = enc->getSpeed();
     // printf("%.2f\n", spd);
}

int main(int argc, char *argv[]){
     int pi;
     float speed, pos;
     int i = 0;
     float sum = 0;

     Encoder* enc;
     pi = pigpio_start(optHost, optPort); /* Connect to Pi. */


     if (pi >= 0){
          enc = new Encoder(pi, optGpioA, optGpioB, -1, 64, 90, optGlitch, optMode, callbackFun);

          thread firstSpd(&enc->calc_speed,enc);
          while(1){
               // usleep(0.001 * 1000000);
               // pos = enc->getPosition();
               // speed = enc->getSpeed();
               // printf("%.2f        %.2f\r\n", pos,speed);

               speed = enc->getSpeed(1);
               float spd = enc->getSpeed(0);

               // average(speed,&i,1000,&sum);
               cout << "Moving Average, Instant: " << speed << ",     " << spd << endl;

          }

          //  std::cout << enc.getPosition() << std::endl;

          pigpio_stop(pi);
     }

     return 0;
}
/**       END Speed Code      */

/** TO COMPILE:

     g++ test_encoder.cpp encoder.cpp -o TestEncoder -lpigpiod_if2 -Wall -pthread

*/
