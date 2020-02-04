#include <unistd.h>
#include <iostream>

#include <pigpiod_if2.h>
#include "drivetrains/dual_roboclaw.h"

using namespace std;

int main(int argc, char *argv[]){
     float target_lin = 0.0, target_rot = 0.0;
     vector<int32_t> cmds;
     // vector<int32_t> cmds1;
     // vector<int32_t> cmds2;

     float dVel = 0.01;
     float dRot = 0.1;

     int pi = pigpio_start(NULL, NULL); /* Connect to Pi. */
     if (pi >= 0){
          DualClaw claws(pi);
          claws.init("/home/pi/devel/robo-commander/config/profiles/dualclaw.config");
          claws.reset_encoders();

          // cmds1 = claws.get_target_speeds(1.0, 0.0);
          // cmds2 = claws.get_target_speeds(0.0, 0.0);

          int err, c;
          bool flag_stop = false;
          while(!flag_stop){
               c = getchar();
               switch(c){
                    case 'q': // Quit
                         flag_stop = true;
                         target_lin = 0.0;
                         target_rot = 0.0;
                         break;
                    case 'w': // Both Sides forward
                         target_lin = target_lin + dVel;
                         printf("Increasing linear velocity to %.3f...\r\n", target_lin);
                         break;
                    case 's': // Both Sides backward
                         target_lin = target_lin - dVel;
                         printf("Decreasing linear velocity to %.3f...\r\n", target_lin);
                         break;
                    case 'a': // Move CCW
                         target_rot = target_rot + dRot;
                         printf("Increasing angular velocity to %.3f...\r\n", target_rot);
                         break;
                    case 'd': // Move CW
                         target_rot = target_rot - dRot;
                         printf("Decreasing angular velocity to %.3f...\r\n", target_rot);
                         break;
                    case 'z': // check speeds
                         // spd1 = claw1->ReadSpeedM1(&status1,&valid1);
                         // spd2 = claw1->ReadSpeedM2(&status2,&valid2);
                         // spd3 = claw2->ReadSpeedM1(&status3,&valid3);
                         // spd4 = claw2->ReadSpeedM2(&status4,&valid4);
                         // printf("V1, V2, V3, V4:     %d   |    %d |    %d  |    %d\r\n",spd1,spd2,spd3,spd4);
                         break;
                    case 'e': // Check PID Params
                         // float kp1, ki1, kd1;
                         // float kp2, ki2, kd2;
                         // float kp3, ki3, kd3;
                         // float kp4, ki4, kd4;
                         // uint32_t qpps1, qpps2, qpps3, qpps4;
                         // // if(claw1->ReadM1VelocityPID(&kp1,&ki1,&kd1,&qpps1))
                         // if(claw1->ReadM1VelocityPID(kp1,ki1,kd1,qpps1))
                         //      printf("Motor1 -- KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp1,ki1,kd1,qpps1);
                         // // if(claw1->ReadM2VelocityPID(&kp2,&ki2,&kd2,&qpps2))
                         // if(claw1->ReadM2VelocityPID(kp2,ki2,kd2,qpps2))
                         //      printf("Motor2 -- KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp2,ki2,kd2,qpps2);
                         // // if(claw2->ReadM1VelocityPID(&kp3,&ki3,&kd3,&qpps3))
                         // if(claw2->ReadM1VelocityPID(kp3,ki3,kd3,qpps3))
                         //      printf("Motor3 -- KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp3,ki3,kd3,qpps3);
                         // // if(claw2->ReadM2VelocityPID(&kp4,&ki4,&kd4,&qpps4))
                         // if(claw2->ReadM2VelocityPID(kp4,ki4,kd4,qpps4))
                         //      printf("Motor4 -- KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp4,ki4,kd4,qpps4);
                         break;
                    case 'r': // STOP
                         printf("Setting speeds to 0...\r\n");
                         target_lin = 0.0;
                         target_rot = 0.0;
                         break;
               }
               cmds = claws.get_target_speeds(target_lin, target_rot);
               claws.drive(cmds);
               usleep(0.01 * 1000000);
               claws.update_encoders(true);
          }


          pigpio_stop(pi);
     }

     return 0;
}
