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
     int width, channel, freq, flag_test;
     float duty;
     char c;

     int pi = pigpio_start(NULL,NULL);
     PCA9685 pwm(pi, bus, add);

     cout << "Please enter what frequency you'd like to try (40Hz - 1000Hz)." << endl;
     cin >> freq;

     pwm.setFrequency(freq);

     cout << "What function would you like to test? 'p' for pulsewidth, or 'd' for dutycycle" << endl;
     cin >> c;

     if(c == 'p')
          flag_test = 1;
     else if(c == 'd')
          flag_test = 0;

     while(1){

          cout << "Which channel would you like to test" << endl;
          cin >> channel;

          if(!flag_test){
               cout << "Please enter the desired duty cycle (0 - 100%)" << endl;
               cin >> duty;
               pwm.setDutyCycle(channel,duty);
          }else{
               cout << "Please enter the desired pulsewidth (in microseconds)" << endl;
               cin >> width;
               pwm.setPulsewidth(channel,width);
          }

          cout << "Would you like to exit? (y) or (n)" << endl;
          cin >> c;
          if(c == 'y')
               break;

     }

     return 0;
}
