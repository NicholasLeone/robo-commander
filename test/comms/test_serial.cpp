#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>
#include <signal.h>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "serial_dev.h"

using namespace std;

int flag_exit = 0;

void my_handler(int s){
	printf("Caught signal %d\n",s);
	flag_exit = 1;
}

int main(int argc, char** argv) {

     struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

     SerialDev ser("/dev/ttyUSB1", 9600);

	int count = 0;
	
	while(!flag_exit){
          // data = ser.readLine();
          cout << ser.readLine() << endl;
	  count++;

	  if(count>0)
		flag_exit=1;

     	}

     string data = "FUCK YOU TOO!\r\n";
     ser.writeLine(data);


     cout << endl;
     return EXIT_SUCCESS;

}
/** To compile

     g++ test_serial.cpp -o TestSerial -lboost_system

*/
