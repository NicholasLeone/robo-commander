#include <unistd.h>
#include <iostream>
#include <thread>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <termio.h>

#include <pigpiod_if2.h>
#include "drivetrains/dual_roboclaw.h"

using namespace std;

bool kbhit(void){
    struct termios original;
    tcgetattr(STDIN_FILENO, &original);

    struct termios term;
    memcpy(&term, &original, sizeof(term));

    term.c_lflag &= ~ICANON;
    tcsetattr(STDIN_FILENO, TCSANOW, &term);

    int characters_buffered = 0;
    ioctl(STDIN_FILENO, FIONREAD, &characters_buffered);

    tcsetattr(STDIN_FILENO, TCSANOW, &original);
    bool pressed = (characters_buffered != 0);
    return pressed;
}

void echoOff(void){
    struct termios term;
    tcgetattr(STDIN_FILENO, &term);

    term.c_lflag &= ~ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
}

void echoOn(void){
    struct termios term;
    tcgetattr(STDIN_FILENO, &term);

    term.c_lflag |= ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
}


bool kill_thread = false;

int main(int argc, char *argv[]){
     std::string dev1;
     std::string dev2;
     if(argc < 3){
          dev1 = "/dev/ttyACM0";
          dev2 = "/dev/ttyACM1";
     } else{
          dev1 = argv[1];
          dev2 = argv[2];
     }

     float target_lin = 0.0, target_rot = 0.0;
     vector<int32_t> cmds;
     // vector<int32_t> cmds1;
     // vector<int32_t> cmds2;

     float dVel = 0.01;
     float dRot = 0.1;

     int pi = pigpio_start(NULL, NULL); /* Connect to Pi. */
     if (pi >= 0){
          int err;
          DualClaw claws(pi);
          err = claws.init(115200, dev1.c_str(), dev2.c_str(), 128, 129);
          // err = claws.init("/home/pi/devel/robo-commander/config/profiles/dualclaw.config");
          if(err < 0){
               return -1;
          }
          // cmds1 = claws.get_target_speeds(1.0, 0.0);
          // cmds2 = claws.get_target_speeds(0.0, 0.0);
          // claws.reset_encoders();
          bool flag_stop = false;
          echoOff();
          int c = '\0';
          while(!flag_stop){
               if (kbhit()){
                    c = getchar();
                    printf("got key \'%c\'\n", c);
                    // if(c==27) { printf("\nuser break\n"); return 1;  }
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
                         case 'f': // check speeds
                              // claws.flip_odom_turn_direction();
                              break;
                         case 'e': // Check PID Params
                              claws.reset_encoders();
                              break;
                         case 'r': // STOP
                              printf("Setting speeds to 0...\r\n");
                              target_lin = 0.0;
                              target_rot = 0.0;
                              claws.reset_odometry();
                              break;
                    }
               }
               cmds = claws.get_target_speeds(target_lin, target_rot);
               claws.drive(cmds);
               claws.update_odometry();
               claws.update_status(true);

               float ppm = claws.get_qpps_per_meter();
          	float wheelbase = claws.get_base_width();
          	float wheel_diamter = claws.get_wheel_diameter();
          	float max_claw_speed = claws.get_max_speed();
          	vector<uint32_t> positions = claws.get_encoder_positions();
          	vector<float> spds = claws.get_motor_speeds();
          	vector<float> dOdom = claws.get_odom_deltas();
          	vector<float> pose = claws.get_pose();
          	vector<float> vels = claws.get_velocities();
          	vector<float> currents = claws.get_currents();
          	vector<float> voltages = claws.get_voltages();

               printf("Motor Speeds (m/s):  %.3f | %.3f  | %.3f  | %.3f \r\n",spds[0],spds[1],spds[2],spds[3]);
     		printf("Encoder Positions (qpps): %d | %d | %d | %d\r\n",positions[0],positions[1],positions[2],positions[3]);
     		printf("Δdistance, ΔYaw, ΔX, ΔY,: %.3f, %.3f, %.3f, %.3f\r\n",dOdom[0], dOdom[1],dOdom[2],dOdom[3]);
     		printf("Current Pose [X (m), Y (m), Yaw (rad)]: %.3f     |    %.3f   |       %.3f\r\n",pose[0],pose[1],pose[2]);
     		// printf("Battery Voltages:     %.3f |    %.3f\r\n",voltages[0], voltages[1]);
     		// printf("Motor Currents:     %.3f |    %.3f |    %.3f |    %.3f\r\n",currents[0], currents[1], currents[2], currents[3]);
     		printf(" =========================================== \r\n");


               usleep(0.1 * 1000000);
          }
          echoOn();

          pigpio_stop(pi);
     }

     return 0;
}
