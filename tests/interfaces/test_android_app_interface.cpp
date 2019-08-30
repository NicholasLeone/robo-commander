#include <iostream>

#include "utilities/utils.h"
#include "interfaces/android_app_interface.h"

using namespace std;

int flag_exit = 0;

void funExit(int s){
     printf("[Ctrl+C] Shutting Down...\r\n");
	flag_exit = 1;
	usleep(1 * 1000000);
}

int main(int argc, char *argv[]){
     int timeoutCnt = 0;

     AndroidInterfaceData recvData;
     AndroidAppInterface* face = new AndroidAppInterface(14500);
     face->mUdp->set_verbose(0);

     attach_CtrlC(funExit);
     printf("Starting up...\r\n");
     usleep(1 * 1000000);
     while(1){
          int err = face->receiveUdpMessage();
          if(err < 0){
               timeoutCnt++;
               printf("[INFO] AndroidAppInterface_Test() --- Timeout Count = %d\r\n",timeoutCnt);
          }

          // printf("[INFO] face->receiveUdpMessage() --- Returned code \'%d\'\r\n",err);
          recvData = face->getReceivedData();
          float v = recvData.normalized_speed * 1.5;
          float w = recvData.normalized_turn_rate * (1.5 / 0.381);
          if((v != 0) || (w != 0)) printf("Velocity1 = %.3f, %.3f\r\n",v,w);

          usleep(0.01 * 1000000);
          if(flag_exit == 1)
               break;
     }
     delete face;

     return 0;
}
