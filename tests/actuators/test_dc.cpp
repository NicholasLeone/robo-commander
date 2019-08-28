#include <unistd.h>
#include <pigpiod_if2.h>
#include "actuators/dc_motor.h"

using namespace std;

float linSpeed = 0;
float angSpeed = 0;
float ang1 = 0;
float ang2 = 0;

int main(int argc, char *argv[]){
     int err;

     int pi = pigpio_start(NULL,NULL);
     DcMotor FR(pi,21,20);
     FR.setSpeed(0);
     sleep(1);
     FR.setSpeed(0.50);
     sleep(1);
     FR.setSpeed(-0.50);
     sleep(1);
     FR.setSpeed(0);

     return 0;
}
