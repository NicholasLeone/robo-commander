#include <iostream>

#include <pigpiod_if2.h>
#include "utilities/utils.h"
#include "robots/swansonV2.h"

using namespace std;

int flag_exit = 0;
SwansonV2* bot;

void funExit(int s){
     printf("[Ctrl+C] Shutting Down...\r\n");
	flag_exit = 1;
	usleep(1 * 1000000);
}

int main(int argc, char *argv[]){

     float max_vel = 1.5;
     float max_omega = max_vel / 0.381;

     int pi = pigpio_start(NULL, NULL); /* Connect to Pi. */
     attach_CtrlC(funExit);

     if (pi >= 0){
          bot = new SwansonV2(pi);

          while(1){
               bot->update_control_interface();
               float v = bot->cmdData.normalized_speed;
          	float w = bot->cmdData.normalized_turn_rate;
               cout << "Controls: " << v << ",          " << w  << endl;
               bot->drive(v, w);
               // bot->update_sensors();
               // vector<float> data = bot->get_sensor_data();
               // bot->add_datalog_entry(data);
               bot->mRelay->mUdp->flush();
               usleep(0.01 * 1000000);
               if(flag_exit == 1){
                    break;
               }
          }
          delete bot;
          pigpio_stop(pi);
     }

     return 0;
}
