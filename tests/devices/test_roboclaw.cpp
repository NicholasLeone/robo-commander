#include <iostream>
#include <unistd.h>
#include <thread>

#include <pigpiod_if2.h>
#include "devices/roboclaw.h"

using namespace std;

RoboClaw* claw1;
RoboClaw* claw2;

void readSpeeds(){
     uint8_t status1, status2, status3, status4;
     bool valid1, valid2, valid3, valid4;
     int32_t spd1, spd2, spd3, spd4;

     while(1){
          usleep(0.01 * 1000000);
          spd1 = claw1->ReadSpeedM1(&status1,&valid1);
          spd2 = claw1->ReadSpeedM2(&status2,&valid2);
          spd3 = claw2->ReadSpeedM1(&status3,&valid3);
          spd4 = claw2->ReadSpeedM2(&status4,&valid4);
          printf("V1, V2, V3, V4:     %d   |    %d |    %d  |    %d\r\n",spd1,spd2,spd3,spd4);
          // printf("Status1, Status2, Status3, Status4:     %d   |    %d |    %d    |    %d\r\n",status1,status2,status3,status4);
          // printf("Valid1, Valid2, Valid3, Valid4:     %d   |    %d |    %d   |    %d\r\n",valid1,valid2,valid3,valid4);
          // printf(" =========================================== \r\n");
     }
}

int main(int argc, char *argv[]){
     uint32_t spd1, spd2, spd3, spd4;
     bool valid1, valid2, valid3, valid4;
     uint8_t status1, status2, status3, status4;

     int h;
     uint32_t go1 = 0;
     uint32_t go2 = -10000;
     int pi = pigpio_start(NULL, NULL); /* Connect to Pi. */

     if(pi >= 0){
          h = serial_open(pi, "/dev/ttyS0",115200,0);
          // h = serial_open(pi, "/dev/ttyS0",57600,0);

          if(h >= 0){
               claw1 = new RoboClaw(pi, h, 128);
               claw2 = new RoboClaw(pi, h, 129);

               // usleep(2 * 1000000);
               // thread speedReader(readSpeeds);

               int err, c;
               bool flag_stop = false;
               while(!flag_stop){
                    c = getchar();
                    switch(c){
                         case 'q': // Quit
                              flag_stop = true;
                              break;
                         case 'w': // Both Sides forward
                              printf("Moving both sides forward...\r\n");
                              claw1->SpeedM1M2(go1,go1);
                              claw2->SpeedM1M2(go1,go1);
                              break;
                         case 's': // Both Sides backward
                              printf("Moving both sides backward...\r\n");
                              claw1->SpeedM1M2(-go1,-go1);
                              claw2->SpeedM1M2(-go1,-go1);
                              break;
                         case 'a': // Move CCW
                              printf("Moving CCW...\r\n");
                              claw1->SpeedM1M2(go1,go1);
                              claw2->SpeedM1M2(-go1,-go1);
                              break;
                         case 'd': // Move CW
                              printf("Moving CW...\r\n");
                              claw1->SpeedM1M2(-go1,-go1);
                              claw2->SpeedM1M2(go1,go1);
                              break;
                         case 'z': // check speeds
                              spd1 = claw1->ReadSpeedM1(&status1,&valid1);
                              // usleep(0.01 * 1000000);
                              spd2 = claw1->ReadSpeedM2(&status2,&valid2);
                              spd3 = claw2->ReadSpeedM1(&status3,&valid3);
                              spd4 = claw2->ReadSpeedM2(&status4,&valid4);
                              printf("V1, V2, V3, V4:     %d   |    %d |    %d  |    %d\r\n",spd1,spd2,spd3,spd4);
                              break;
                         case 'e': // Check PID Params
                              float kp1, ki1, kd1;
                              float kp2, ki2, kd2;
                              float kp3, ki3, kd3;
                              float kp4, ki4, kd4;
                              uint32_t qpps1, qpps2, qpps3, qpps4;
                              // if(claw1->ReadM1VelocityPID(&kp1,&ki1,&kd1,&qpps1))
                              if(claw1->ReadM1VelocityPID(kp1,ki1,kd1,qpps1))
                                   printf("Motor1 -- KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp1,ki1,kd1,qpps1);
                              // if(claw1->ReadM2VelocityPID(&kp2,&ki2,&kd2,&qpps2))
                              if(claw1->ReadM2VelocityPID(kp2,ki2,kd2,qpps2))
                                   printf("Motor2 -- KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp2,ki2,kd2,qpps2);
                              // if(claw2->ReadM1VelocityPID(&kp3,&ki3,&kd3,&qpps3))
                              if(claw2->ReadM1VelocityPID(kp3,ki3,kd3,qpps3))
                                   printf("Motor3 -- KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp3,ki3,kd3,qpps3);
                              // if(claw2->ReadM2VelocityPID(&kp4,&ki4,&kd4,&qpps4))
                              if(claw2->ReadM2VelocityPID(kp4,ki4,kd4,qpps4))
                                   printf("Motor4 -- KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp4,ki4,kd4,qpps4);
                              break;
                         case '1': // Increase speed
                              go1 = go1 + 500;
                              printf("Increased speed to %d\r\n",go1);
                              break;
                         case '2': // Decrease speed
                              go1 = go1 - 500;
                              printf("Decreased speed to %d\r\n",go1);
                              break;
                         case '3': // STOP
                              printf("Stopping...\r\n");
                              go1 = 0;
                              claw1->SpeedM1M2(go1,go1);
                              claw2->SpeedM1M2(go1,go1);
                              break;
                    }
                    usleep(0.01 * 1000000);
               }
               // Shutdown everything
               claw1->SpeedM1M2(0,0);
               claw2->SpeedM1M2(0,0);
          }
          int err = serial_close(pi, h);
          usleep(0.1 * 10000000);
          pigpio_stop(pi);
     }
     // usleep(1 * 10000000);
     cout << "DONE!" << endl;

     return 0;
}

/** TO COMPILE:

     g++ -w test_roboclaw.cpp -o TestClaw -lpigpiod_if2 -Wall -pthread

*/
