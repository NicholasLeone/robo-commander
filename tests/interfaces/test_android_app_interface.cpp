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
     AndroidAppInterface* iface;
     iface = new AndroidAppInterface();
     iface->mUdp->set_verbose(0);

     attach_CtrlC(funExit);

     while(1){
          iface->receiveUdpMessage();

          usleep(0.01 * 1000000);
          if(flag_exit == 1)
               break;
     }
     delete iface;

     return 0;
}
