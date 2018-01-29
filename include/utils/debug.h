#ifndef DEBUG_H_
#define DEBUG_H_

#include "base/params.h"

// UDP Sending Overloads
// int sendUdp(int _port, char* _add, CommunicationHeaderByte* header, Sim_Msg_IMUData data);
// int sendUdp(int _port, char* _add, CommunicationHeaderByte* header, Sim_Msg_GPSData data);
// int sendUdp(int _port, char* _add, CommunicationHeaderByte* header, Sim_Msg_LidarData data);

void printUdpHeader(CommunicationHeaderByte* header);
void printImu(Sim_Msg_IMUData data);
void printGps(Sim_Msg_GPSData data);
void printLidar(Sim_Msg_LidarData data);

#endif /** DEBUG_H_ */
