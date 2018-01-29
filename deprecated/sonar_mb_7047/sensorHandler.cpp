#include <stdio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <iostream>
#include <unistd.h>

#include "./sensorHandler.h"
#include "./mb7047.h"
using namespace std;

#define DEVICE_ADDR 0x70
#define DEVICE_BUS 1
int main(){

	float distance;
	mb7047 *Sonar = new mb7047(DEVICE_BUS, DEVICE_ADDR);
	while(1)
	{
		distance = Sonar->getDistance();

		//cout<<"Distance to Object(cm) = "<<distance<<endl;

	}
	return 0;

}

