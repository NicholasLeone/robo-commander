#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <math.h>

#include <pigpiod_if2.h>
#include "devices/pca9685.h"

using namespace std;

int main(int argc, char *argv[]){
     int err;
     int bus = 1;
     int add = 0x70;
     int width, channel, freq, flag_exit;
     float duty;
     char c;

     int pi = pigpio_start(NULL,NULL);
     PCA9685 pwm(pi, bus, add);

     flag_exit = 0;
     freq = 50;
     pwm.setFrequency(freq);

     cout << "LED Color Test:" << endl;
     cout << "Enter: 	" << endl;
	cout << "	r - Red" << endl;
	cout << "	g - Green" << endl;
	cout << "	b - Blue" << endl;
	cout << "	w - White" << endl;
	cout << "	k - Off" << endl;
	cout << "	p - Purple" << endl;
	cout << "	y - Yellow" << endl;
	cout << "	q - Quit" << endl;
	cout << "Follow by return key in each case." << endl;
	cout << "Enter: ";

     while(!flag_exit){
          cin >> c;
          switch(c){
               case 'R':
               case 'r':
                    cout << "Red..." << endl;
                    pwm.setDutyCycle(0,100.0);
                    pwm.setDutyCycle(1,0);
                    pwm.setDutyCycle(2,0);
                    break;
               case 'G':
               case 'g':
                    cout << "Green..." << endl;
                    pwm.setDutyCycle(0,0);
                    pwm.setDutyCycle(1,100.0);
                    pwm.setDutyCycle(2,0);
                    break;
               case 'B':
               case 'b':
                    cout << "Blue..." << endl;
                    pwm.setDutyCycle(0,0);
                    pwm.setDutyCycle(1,0);
                    pwm.setDutyCycle(2,100.0);
                    break;
               case 'W':
               case 'w':
                    cout << "White..." << endl;
                    pwm.setDutyCycle(0,100.0);
                    pwm.setDutyCycle(1,100.0);
                    pwm.setDutyCycle(2,100.0);
                    break;
               case 'K':
               case 'k':
                    cout << "Off..." << endl;
                    pwm.setDutyCycle(0,0);
                    pwm.setDutyCycle(1,0);
                    pwm.setDutyCycle(2,0);
                    break;
               case 'P':
               case 'p':
                    cout << "Purple..." << endl;
                    pwm.setDutyCycle(0,100.0);
                    pwm.setDutyCycle(1,0);
                    pwm.setDutyCycle(2,100.0);
                    break;
               case 'Y':
               case 'y':
                    cout << "Yellow..." << endl;
                    pwm.setDutyCycle(0,100.0);
                    pwm.setDutyCycle(1,100.0);
                    pwm.setDutyCycle(2,0);
                    break;
               case 'Q':
               case 'q':
                    flag_exit = 1;
                    break;
               default:
                    cout << "Color Not Supported Yet. Please try Again." << endl;
          }


          // cout << "Would you like to exit? (y) or (n)" << endl;
          // cin >> c;
          // if(c == 'y')
          //      break;

     }

     return 0;
}
