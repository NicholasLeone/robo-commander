#define TEST_DEBUG 0
#include "actuators.h"
#include "motors.h"
// #include "motor-driver.h"
// #include "pca9685/PWM-Driver-PCA9685.h"
#include <stdio.h>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <ncurses.h>

// #include <sys/types.h>
// #include <sys/socket.h>
// #include <net/if.h>
// #include <netinet/in.h>
// #include <arpa/inet.h>
// #include <netdb.h>
// #include <stdio.h>
// #include <string.h>
// #include <unistd.h>
// #include <fcntl.h>
// #include <sys/select.h>

typedef struct Msg_Command_RC3D{
    int32_t yaw;//radians 1 <
    int32_t pitch;//radians
    int32_t speed;//m/s
}Msg_Command_RC3D;


class UDPData {

public:
    int fd;
    int sock;
    int in_port;
    int dev_address;
    int out_port;
    char* file;
    socklen_t sock_len;
    struct sockaddr_in remAddr;
    struct sockaddr_in myAddr;
    struct hostent * hp;
    fd_set read_fds;
    fd_set write_fds;
    struct timeval tv;
};

static UDPData* my_data;
static UDPData* sender_data;
static char sink_ip[100];
//static const int recv_port = 49000;
//static const int send_port = 49000;

static const int recv_port = 14500;
static const int send_port = 14500;
static char buf[1000];


static int _udpOpen(UDPData* ifdata, int port, char* address){
    int err;
    int flags;

    bzero(&ifdata->myAddr, sizeof(sockaddr_in));

    ifdata->sock = socket(AF_INET, SOCK_DGRAM, 0);

    if(ifdata->sock == 0)
        return -1;

    ifdata->in_port = port;

    if(!address){
        ifdata->myAddr.sin_addr.s_addr = INADDR_ANY;
    }
    else{
        ifdata->myAddr.sin_addr.s_addr = inet_addr(address);
    }
    ifdata->myAddr.sin_family = AF_INET;
    ifdata->myAddr.sin_port = htons(ifdata->in_port);

    flags = fcntl(ifdata->sock, F_GETFL);
    // UNCOMMENTED BEFORE
    flags |= O_NONBLOCK;
    fcntl(ifdata->sock, F_SETFL, flags);
    //fcntl(socket_fd, F_SETFL, O_NONBLOCK); //set socket to non-blocking
    // clear the set ahead of time
    FD_ZERO(&ifdata->read_fds);
    FD_ZERO(&ifdata->write_fds);

    bzero(&(ifdata->myAddr.sin_zero),8);

    // Bind the socket to port 14555 - necessary to receive packets from qgroundcontrol
    if (-1== bind(ifdata->sock, (struct sockaddr *) &ifdata->myAddr,sizeof(struct sockaddr))) {
        perror("error bind failed");
        close(ifdata->sock);
        return -2;
    }

    //setsockopt(ifdata->sock, SO_REUSEADDR);

    FD_SET(ifdata->sock, &ifdata->write_fds);
    FD_SET(ifdata->sock, &ifdata->read_fds);

    return 0;
}
static int _udpClose(UDPData* data){

    if(data == NULL)
        return -1;

    close(data->sock);

    return 0;

    return 0;
}


static int _udpWrite(UDPData* data, char* buf, int numBytes, char* address, int port){
    int n;

    int receive;

    if(data == NULL)
        return -1;

    if(buf == NULL)
        return -2;

    if(numBytes == 0)
        return -3;

    if(!address)
        return -4;

    memset(&data->remAddr, 0, sizeof(struct sockaddr_in));
    data->remAddr.sin_family = AF_INET;
    data->remAddr.sin_addr.s_addr = inet_addr(address);
    data->remAddr.sin_port = htons(port);



    n=sendto(data->sock,(void*)buf,numBytes,0,(struct sockaddr *)&data->remAddr,sizeof(struct sockaddr_in));

    if(n<0)	{
        perror("Sending message");
        return -7;
    }

    return 0;
}



static int _udpRead(UDPData* data, char* buf, int numBytes){
    int n;
    socklen_t len;

    int receive;

    if(data == NULL)
        return -1;

    if(buf == NULL)
        return -2;

    if(numBytes == 0)
        return -3;

    //receive = select(ifdata->sock + 1, &ifdata->read_fds, NULL, NULL, &ifdata->tv);

    if (receive == -1){
        //printf("select error read\n");
        //return -4;
    }
    else if (receive == 0)	{ //timeout
        //printf("timeout read\n");
        //return 0;
    }

    n = recvfrom(data->sock,(void*)buf,numBytes,0,(struct sockaddr *)&data->remAddr,&(len=sizeof(struct sockaddr_in)));

	int i; for(i=0;i<numBytes;i++){
          std::cout <<  (int)buf[i];
     }

    //FD_SET(ifdata->sock, &ifdata->read_fds);

    if (n<-1)
    {
        perror("recvfrom");
        return -6;
    }

    return n;
}



static Msg_Command_RC3D controls;

static void initReadControls(){
	my_data = new UDPData;

	_udpOpen(my_data, recv_port, NULL);

    memset(buf, 0, sizeof(buf));

    memset(sink_ip, 0, sizeof(sink_ip));
}

static void readControls(Msg_Command_RC3D* data){
	int num;
	while(_udpRead(my_data, buf, sizeof(controls)+2) > 0){}
	memcpy(data, &buf[2], sizeof(controls));
}



static Motor* motorArray[4];

float linSpeed = 0;
float angSpeed = 0;
float ang1 = 0;
float ang2 = 0;

using namespace std;

SunfounderMotor* motor1;
SunfounderMotor* motor2;
SunfounderMotor* motor3;
SunfounderMotor* motor4;

void drive(float v, float w) {
     float centerToWheelRadius = 0.219;

     /**   Differential Drive Drive Equations     */
     float vLeft = v - w * centerToWheelRadius;
     float vRight = v + w * centerToWheelRadius;

     // TODO: Determine motor configuration
     // Left Motors
     motor1->setSpeed(vLeft);
     motor2->setSpeed(-vLeft);
     // Right Motors
     motor3->setSpeed(-vRight);
     motor4->setSpeed(vRight);

}

void parseChar(int c){
     switch(c){
          case 'q': // Increase Linear Speed
               linSpeed = linSpeed + 0.01;
               break;
          case 'a': // Decrease Linear Speed
               linSpeed = linSpeed - 0.01;
               break;
          case 'w': // Increase Angular Speed
               angSpeed = angSpeed + 0.01;
               break;
          case 's': // Decrease Angular Speed
               angSpeed = angSpeed - 0.01;
               break;
          case 'e': // Servo 1 Up
               ang1 = ang1 + 1;
               break;
          case 'd': // Servo 1 Down
               ang1 = ang1 - 1;
               break;
          case 'r': // Servo 2 Up
               ang2 = ang2 + 1;
               break;
          case 'f': // Servo 2 Down
               ang2 = ang2 - 1;
               break;
          case 'k': // STOP
               linSpeed = 0;
               angSpeed = 0;
               break;
     }

     cout << "Linear Speed, Angular Speed: " << linSpeed << "    |    " << angSpeed << "\r\n";

}



int main(int argc, char *argv[]){
     int err, c;

     err = _initMotors(4, &motorArray[0]);

     motor1 = ((SunfounderMotor*) motorArray[0]);
     motor2 = ((SunfounderMotor*) motorArray[1]);
     motor3 = ((SunfounderMotor*) motorArray[2]);
     motor4 = ((SunfounderMotor*) motorArray[3]);

     initReadControls();
     this_thread::sleep_for (chrono::seconds(1));
     //initscr();
     //printw("Begin Bot Control.... ");
     while(1){
	  readControls(&controls);
	  float accel = (float) controls.speed / 1000000;
	  float omega = (float) controls.yaw / 1000000;

/**
	  if(accel == 2){
	 	linSpeed = 0;
		angSpeed = 0;
	  } else {
	  	linSpeed = accel;
	  	angSpeed = omega;
		if(linSpeed > 1)
			linSpeed = 1;
		if(angSpeed > 3)
			angSpeed = 3;

	  }
          */
	  cout << "Linear Speed, Angular Speed: " << accel << "    |    " << omega << "\r\n";
	  drive(linSpeed, angSpeed);
     }

     //endwin();
     return 0;
}


/** TO COMPILE:

     g++ test_motors.cpp motor-driver.cpp -o TestMotor -lpigpiod_if2 -Wall -pthread

     g++ test_motors.cpp motor-driver.cpp ./pca9685/PWM-Driver-PCA9685.cpp ./pca9685/pwm-pca9685-user.c -o TestRC -lpigpiod_if2 -Wall -pthread -std=c++11

*/
