#include <iostream>
#include <unistd.h>
#include <thread>

#include <pigpiod_if2.h>
#include "roboclaw.h"

using namespace std;

RoboClaw* claw1;
RoboClaw* claw2;

void readSpeeds(){
     uint8_t status1, status2, status3, status4;
     bool valid1, valid2, valid3, valid4;
     int32_t spd1, spd2, spd3, spd4;

     while(1){
          usleep(0.1 * 1000000);
          spd1 = claw1->ReadSpeedM1(&status1,&valid1);
          spd2 = claw1->ReadSpeedM2(&status2,&valid2);
          spd3 = claw2->ReadSpeedM1(&status3,&valid3);
          spd4 = claw2->ReadSpeedM2(&status4,&valid4);
          printf("V1, V2, V3, V4:     %d   |    %d |    %d  |    %d\r\n",spd1,spd2,spd3,spd4);
          // printf("Status1, Status2, Status3, Status4:     %d   |    %d |    %d    |    %d\r\n",status1,status2,status3,status4);
          // printf("Valid1, Valid2, Valid3, Valid4:     %d   |    %d |    %d   |    %d\r\n",valid1,valid2,valid3,valid4);
     }
}


int main(int argc, char *argv[]){

     uint32_t qpps1, qpps2, qpps3, qpps4;
     float kp, ki, kd;
     uint32_t go1 = 10000;
     uint32_t go2 = -10000;
     int pi = pigpio_start(NULL, NULL); /* Connect to Pi. */
     int h;

     if(pi >= 0){
          h = serial_open(pi, "/dev/ttyS0",115200,0);

          if(h >= 0){
               claw1 = new RoboClaw(pi, h, 128);
               claw2 = new RoboClaw(pi, h, 129);

               claw1->ReadM1VelocityPID(kp,ki,kd,qpps1);
               printf("KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp,ki,kd,qpps1);
               claw1->ReadM2VelocityPID(kp,ki,kd,qpps2);
               printf("KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp,ki,kd,qpps2);
               claw2->ReadM1VelocityPID(kp,ki,kd,qpps3);
               printf("KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp,ki,kd,qpps3);
               claw2->ReadM2VelocityPID(kp,ki,kd,qpps4);
               printf("KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp,ki,kd,qpps4);

               usleep(2 * 1000000);
               thread speedReader(readSpeeds);

               claw1->SpeedM1M2(go1,go1);
               claw2->SpeedM1M2(go2,go2);
               usleep(10 * 1000000);
               claw1->SpeedM1M2(0,0);
               claw2->SpeedM1M2(0,0);
               int err = serial_close(pi, h);
          }
          usleep(1 * 10000000);
          pigpio_stop(pi);
     }
     usleep(1 * 10000000);
     cout << "DONE!" << endl;

     return 0;
}

/** TO COMPILE:

     g++ -w test_roboclaw.cpp -o TestClaw -lpigpiod_if2 -Wall -pthread

*/
