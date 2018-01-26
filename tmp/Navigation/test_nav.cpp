#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/select.h>
#include <stdlib.h>
#include <pthread.h>
#include "waypoint_follower.h"
#include <math.h>

#define TEST_CONTROL 0
#define _USE_MATH_DEFINES

#ifdef TEST_CONTROL
#include "../Motors/actuators.h"
#include "../Motors/motors.h"
#include "../Motors/motor-driver.h"
#include "../Motors/pca9685/PWM-Driver-PCA9685.h"
#endif

typedef struct AttitudeData{
   int32_t yaw;
   int32_t pitch;
   int32_t roll;
   int32_t p1;
   int32_t p2;
   int32_t p3;
   int32_t p4;
}AttitudeData;

typedef struct Msg_GPSData{
	uint32_t gpstime;
	int32_t heading;
	uint32_t speed;
	uint32_t accuracy;
	int32_t lat;
	int32_t lon;
	uint32_t alt;
}Msg_GPSData;

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

/** Waypoint Following Example on Differential drive robot:

int main(){
     fmat curPose;
     fmat curTarget;
     fmat history;
     fmat controls;
     fmat path << 40.102142 << -88.22735 << endr
	       << 40.102140 << -88.22692 << endr
	       << 40.101810 << -88.22691 << endr
	       << 40.101833 << -88.22734 << endr;

     Agent myBot();
     WaypointFollower wf(path, 0.1);
     Sensor gps();

     // Record initial Pose and controls
     curPose = gps.read();
     controls << 0 << 0 << endr;
     history = wf.updateHistory(history,curPose,controls);
     history.print("INITIAL HISTORY SAVED: [X, Y, Yaw, CmdV, CmdAngSpeed] ");

     // Control Agent until last waypoint reached
     while(!wf.goalReached){
          cout << "Distance to Goal: " << wf.destDist << endl;

          // Update current pose from sensor data
          curPose = gps.read();
          curTarget = wf.updateTarget(curPose);

          // Calculate controls to get to the target waypoint
          controls = wf.updateControls(curTarget,curPose);

          history = wf.updateHistory(prevHistory,curPose,controls);

          // Command physical Agent's actuators
          myBot.drive(controls);

     }

     history.save("VAR_Robot_History.csv", csv_ascii);

}

*/

static UDPData* my_data;
static UDPData* sender_data;
static char sink_ip[100];
static const int recv_port = 49000;
static const int send_port = 49000;
static char buf[1000];

static int _udpFlush(UDPData* data){
    int n;
    int flags;
    int new_flags;
    int chunk_size = sizeof(int);
    char chunk[chunk_size];
    socklen_t len =sizeof(data->remAddr);

    if(data == NULL)
        return -1;

    flags = fcntl(data->sock, F_GETFL);
    new_flags = flags | O_NONBLOCK;
    fcntl(data->sock, F_SETFL, new_flags);

    do{
        n = recvfrom(data->sock,(void*)chunk,chunk_size,0,(struct sockaddr *)&data->remAddr, &len);
        if(n<-1){
            return -2;
        }
    }while(n==chunk_size);

    if(new_flags != flags){
        fcntl(data->sock, F_SETFL, flags);
    }

    return 0;
}
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
    //flags |= O_NONBLOCK;
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
          // std::cout <<  (int)buf[i] << ", ";
     }
	// std::cout <<  endl;

    //FD_SET(ifdata->sock, &ifdata->read_fds);

    if (n<-1)
    {
        perror("recvfrom");
        return -6;
    }


    return n;
}


static AttitudeData attData;
static Msg_GPSData gpsData;

static void initReadGPS(){
	my_data = new UDPData;

	_udpOpen(my_data, recv_port, NULL);

    memset(buf, 0, sizeof(buf));

    memset(sink_ip, 0, sizeof(sink_ip));
}

static void readGPS(Msg_GPSData* data){

	while(1){
		int numBytes = _udpRead(my_data, buf, sizeof(gpsData)+2);
		memcpy(data, &buf[2], sizeof(gpsData));
		if(numBytes == 0)
			break;
	}
}

static void readAttitude(AttitudeData* data){
     	//_udpFlush(my_data);

	while(1){
		int numBytes = _udpRead(my_data, buf, sizeof(attData)+2);
		memcpy(data, &buf[2], sizeof(attData));
		if(numBytes == 0)
			break;
	}

}

static void readUDP(Msg_GPSData* gpData, AttitudeData* aData){

     while(1){
	  //_udpFlush(my_data);
          int numBytes = _udpRead(my_data, buf, sizeof(attData)+2);
	cout << "bytes: " << numBytes << endl;	

          if(buf[1] == 8)
               memcpy(gpData, &buf[2], sizeof(gpsData));
          else //if(buf[1] == 12)
               memcpy(aData, &buf[2], sizeof(attData));

          if(numBytes == 0)
               break;

     }

}

#ifdef TEST_CONTROL

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
#endif

int main(int argc, char *argv[]){
     fmat path, curPose, history, controls, curTarget;
     float latitude, longitude;
     float heading, speed;

     // TODO: for later usage in kinematic equations
     int timeMs;

#ifdef TEST_CONTROL
	int err = _initMotors(4, &motorArray[0]);
     motor1 = ((SunfounderMotor*) motorArray[0]);
     motor2 = ((SunfounderMotor*) motorArray[1]);
     motor3 = ((SunfounderMotor*) motorArray[2]);
     motor4 = ((SunfounderMotor*) motorArray[3]);
#endif

     path << 40.102142 << -88.22735 << endr
	       << 40.102140 << -88.22692 << endr
	       << 40.10181 << -88.22691 << endr
	       << 40.101833 << -88.22734 << endr;
     initReadGPS();
     readAttitude(&attData);
     float yaw_bias = (float) attData.yaw / 1000000;


     WaypointFollower wf(path, 0.000005);

     curPose << 0 << 0 << 0 << endr;
     controls << 0 << 0 << endr;
     history = wf.updateHistory(history,curPose,controls);
     //history.print("INITIAL HISTORY SAVED: [X, Y, Yaw, CmdV, CmdAngSpeed] ");

     while(1){

          usleep(100000);
          readUDP(&gpsData, &attData);
		//readGPS(&gpsData);
		//readAttitude(&attData);
		latitude = (float) gpsData.lat / 1000000;
		longitude = (float) gpsData.lon / 1000000;
		heading = (float) attData.yaw / 1000000;
          heading = heading * (M_PI/180);// - yaw_bias;
		// cout << "YEHAW: " << heading << endl;
          curPose << latitude << longitude << heading << endr;
		curPose.print("GPS DATA: ");
		curTarget = wf.updateTarget(curPose);
		cout << "Distance to Goal: " << wf.destDist << endl;

		controls = wf.updateControls(curTarget, curPose);
		controls.print("CONTROLS: ");

		float v = as_scalar(controls(0));
		float w = as_scalar(controls(1));
		//drive(v, w);


     }

     /**
     // Control Agent until last waypoint reached
     while(!wf.goalReached){
          cout << "Distance to Goal: " << wf.destDist << endl;

          // Update current pose from sensor data
          curPose = gps.read();
          curTarget = wf.updateTarget(curPose);

          // Calculate controls to get to the target waypoint
          controls = wf.updateControls(curTarget,curPose);

          history = wf.updateHistory(prevHistory,curPose,controls);

          // Command physical Agent's actuators
          //myBot.drive(controls);

     }
     */
     // curTarget = wf.updateTarget(curPose);
     // //curTarget.print("New Target:");
     //
     // controls = wf.updateControls(curTarget, curPose);
     // controls.print("New Controls:");
     // cout << "Distance to Goal: " << wf.destDist << endl;
     //
     // history = wf.updateHistory(history,curPose,controls);
     // history.print("New History: ");

     return 0;
}

/** TO COMPILE

	NAVIGATION ONLY:

     g++ test_nav.cpp waypoint_follower.cpp -o TestNav -larmadillo

	NAV + CONTROLS:

	g++ test_nav.cpp waypoint_follower.cpp ../Motors/motor-driver.cpp ../Motors/pca9685/PWM-Driver-PCA9685.cpp ../Motors/pca9685/pwm-pca9685-user.c -o TestNav2 -larmadillo -lpigpiod_if2 -Wall -pthread

*/
