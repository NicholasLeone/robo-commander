#include <iostream>
#include <unistd.h>
#include <thread>
#include <atomic>

#include "devices/roboclaw_unix.h"

using namespace std;

RoboclawUnix* claw1;
bool kill_thread = false;
bool thread_on = false;

void readSpeeds(){
     uint8_t status1, status2, status3, status4;
     bool valid1, valid2, valid3, valid4;
     int loop_cnt = 0;
     int32_t spd1 = 0, spd2 = 0;
     while(!kill_thread){
          usleep(0.1 * 1000000);
          if(thread_on){
               spd1 = claw1->ReadSpeedM1(&status1,&valid1);
               spd2 = claw1->ReadSpeedM2(&status2,&valid2);
               loop_cnt++;
               printf("Encoders #%d: V1, V2:     %d   |    %d \r\n",loop_cnt, spd1,spd2);
          }
     }
}

int main(int argc, char *argv[]){
     std::thread readThread;
     uint32_t go1 = 0;
     bool valid1, valid2;
     uint8_t status1, status2;
     uint32_t spd1 = 0, spd2 = 0;

     const char *port = "/dev/ttyACM0";
     uint8_t address = 128;
     uint32_t timeout = 1000; // 1 second
     bool doack = false; // do ack for writes
     if (argc == 2) {
          port = argv[1];
          // address = strtoul(argv[2], nullptr, 0);
     }

     claw1 = new RoboclawUnix(port, address, timeout, doack);
     readThread = std::thread(readSpeeds);

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
                    // claw1->DutyAccelM1M2((uint16_t) 0, (uint32_t) go1, (uint16_t) 0, (uint32_t) go1);
                    break;
               case 's': // Both Sides backward
                    printf("Moving both sides backward...\r\n");
                    claw1->SpeedM1M2(-go1,-go1);
                    break;
               case 'a': // Move CCW
                    printf("Moving CCW...\r\n");
                    claw1->SpeedM1M2(go1,go1);
                    break;
               case 'd': // Move CW
                    printf("Moving CW...\r\n");
                    claw1->SpeedM1M2(-go1,-go1);
                    break;
               case 'z': // check speeds
                    thread_on = !thread_on;
                    break;
               case 'x': // check speeds
                    spd1 = claw1->ReadSpeedM1(&status1,&valid1);
                    spd2 = claw1->ReadSpeedM2(&status2,&valid2);
                    printf("V1, V2:     %d   |    %d\r\n",spd1,spd2);
                    break;
               case 'e': // Check PID Params
                    float kp1, ki1, kd1;
                    float kp2, ki2, kd2;
                    uint32_t qpps1, qpps2;
                    // if(claw1->ReadM1VelocityPID(&kp1,&ki1,&kd1,&qpps1))
                    if(claw1->ReadM1VelocityPID(kp1,ki1,kd1,qpps1)) printf("Motor1 -- KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp1,ki1,kd1,qpps1);
                    if(claw1->ReadM2VelocityPID(kp2,ki2,kd2,qpps2)) printf("Motor2 -- KP, KI, KD, QPPS:     %.3f   |    %.3f |    %.3f    |    %d\r\n",kp2,ki2,kd2,qpps2);
                    break;
               case '1': // Increase speed
                    go1 = go1 + 100;
                    printf("Increased speed to %d\r\n",go1);
                    break;
               case '2': // Decrease speed
                    go1 = go1 - 100;
                    printf("Decreased speed to %d\r\n",go1);
                    break;
               case '3': // STOP
                    printf("Stopping...\r\n");
                    go1 = 0;
                    claw1->SpeedM1M2(go1,go1);
                    // claw2->SpeedM1M2(go1,go1);
                    break;
               case 'v': // Decrease speed
                    char version[2000];
                    claw1->ReadVersion(version);
                    printf("ROBOCLAW VERSION: %s\r\n", version);
                    break;
               case 'V': // Decrease spee
                    claw1->set_verbosity(0);
                    break;
               case 'T': // Decrease spee
                    bool valid = false;
                    uint8_t status = 0;
                    printf("Starting Tests.\r\n");
                    claw1->set_verbosity(4);
                    claw1->ResetEncoders();
                    claw1->DutyM1((uint16_t) 5);
                    claw1->DutyM2((uint16_t) 50);
                    // usleep(10.0 * 1000000);
                    claw1->DutyM1M2(0, 0);
                    uint32_t counts = claw1->ReadEncM1( &status, &valid);
                    counts = claw1->ReadEncM2( &status, &valid);
                    int32_t spd = claw1->ReadSpeedM1(&status,&valid);
                    spd = claw1->ReadSpeedM2(&status,&valid);

                    claw1->SpeedM1((uint32_t) 100);
                    claw1->SpeedM2((uint32_t) 1000);
                    // usleep(10.0 * 1000000);
                    claw1->SpeedM1M2(0,0);
                    uint32_t counts1 = 0, counts2 = 0;
                    bool valid_encoders = claw1->ReadEncoders(counts1,counts2);
                    claw1->set_verbosity(0);
                    printf("Stopping Tests.\r\n");

                    break;
          }
          usleep(0.01 * 1000000);
     }
     // Shutdown everything
     claw1->SpeedM1M2(0,0);
     kill_thread = true;

     cout << "Closing Read Thread..." << endl;
     if(readThread.joinable()){ readThread.join(); }
     cout << "DONE!" << endl;
     return 0;
}
