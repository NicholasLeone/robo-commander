#include <thread>
#include <chrono>
#include <iostream>

#include "actuators/servo.h"

static Motor* servoArray[2];
Servo* servo1;
Servo* servo2;
float ang1 = 0;
float ang2 = 0;


using namespace std;


void parseChar(int c){
     switch(c){
          case 'q': // Servo 1 Up
               ang1 = ang1 + 1;
               break;
          case 'a': // Servo 1 Down
               ang1 = ang1 - 1;
               break;
          case 'w': // Servo 2 Up
               ang2 = ang2 + 1;
               break;
          case 's': // Servo 2 Down
               ang2 = ang2 - 1;
               break;
	  case '1':
	       ang1 = -60;
	       ang2 = -60;
	       break;
	  case '2':
	       ang1 = 60;
	       ang2 = -60;
	       break;
	  case '3':
	       ang1 = 0;
	       ang2 = 56;
	       break;
	  case '4':
	       ang1 = 0;
	       ang2 = -61;
	       break;
          case 'k': // STOP
               ang1 = 0;
               ang2 = 0;
               break;
     }

     cout << "Angle 1, Angle 2: " << ang1 << "    |    " << ang2 << endl;

}


int main(int argc, char *argv[]){
     int err, c;

     err = _initServos(2, &servoArray[0]);

     servo1 = ((Servo*) servoArray[0]);
     servo2 = ((Servo*) servoArray[1]);

     this_thread::sleep_for (chrono::seconds(1));

     while(1){
          c = getchar();
          parseChar(c);
          servo1->setAngle(ang1);
          servo2->setAngle(ang2);
     }

     return 0;
}
/** To Compile:

	g++ test_servo.cpp servo.cpp ../Motors/pca9685/PWM-Driver-PCA9685.cpp ../Motors/pca9685/pwm-pca9685-user.c -o TestServo -lpigpiod_if2 -Wall -pthread -std=c++11

*/
