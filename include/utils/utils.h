#ifndef UTILS_H_
#define UTILS_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <vector>
#include <string.h>
#include <map>
#include <functional>
#include "base/params.h"

using namespace std;

int convertSpdRatio2Pulse(float spd_ratio, int max, int min, int neutral);
void LoadInitialVariables(const string &fileName, map<string, float> &variables);
void LoadStringVariables(const string &fileName, map<string, string> &variables);
int countData(string s, char delimiter);
int countLines(const string &fileName);
int extract_bit(int inputByte, int bitLocation);
int extract_bits(int inputByte, int msb, int lsb);
float unpackFloat(char* buffer, int *i);
vector<float> parseFloat(string s, string delimiter);
void attach_CtrlC(void_int_fun func2call);
void attach_CtrlZ(void_int_fun func2call);

void printUdpHeader(CommunicationHeaderByte* header);
void printImu(Sim_Msg_IMUData data);
void printGps(Sim_Msg_GPSData data);
void printLidar(Sim_Msg_LidarData data);

// UDP Sending Overloads
// int sendUdp(int _port, char* _add, CommunicationHeaderByte* header, Sim_Msg_IMUData data);
// int sendUdp(int _port, char* _add, CommunicationHeaderByte* header, Sim_Msg_GPSData data);
// int sendUdp(int _port, char* _add, CommunicationHeaderByte* header, Sim_Msg_LidarData data);

#endif /* UTILS_H_ */
