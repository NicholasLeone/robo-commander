#ifndef UTILITIES_UTILS_H_
#define UTILITIES_UTILS_H_

#include <vector>
#include <map>
#include <armadillo>
#include "base/definitions.h"
#include "yaml-cpp/yaml.h"

using namespace std;
using namespace arma;

float convertRadians2Degrees(float angle);
float convertDegrees2Radians(float angle);
int convertSpdRatio2Pulse(float spd_ratio, int max, int min, int neutral);
void LoadInitialVariables(const string &fileName, map<string, float> &variables);
void LoadStringVariables(const string &fileName, map<string, string> &variables);
int extract_bit(int inputByte, int bitLocation);
int extract_bits(int inputByte, int msb, int lsb);
float unpackFloat(char* buffer, int *i);
int countData(string s, char delimiter);
int countLines(const string &file);

vector<float> parseFloat(string s, string delimiter);
vector<int> get_csv_size(const string &file);
vector<vector<float>> csv_to_array(const string &file);
vector<vector<float>> csv_extract_columns(const string &file);

fmat csv_to_matrix(const string &file);

fmat Ci2b(float angles[3]);

template<typename T> void print_vector(string header, vector<T> vec);
template<typename T> void print_vectors(string header, vector< vector<T> > vecs);

void attach_CtrlC(void_int_fun func2call);
void attach_CtrlZ(void_int_fun func2call);

template<typename... Args>
std::string format(const char* format, Args... args ){
     int length = std::snprintf(nullptr, 0, format, args...);
     assert(length >= 0);

     char* buf = new char[length + 1];
     std::snprintf(buf, length + 1, format, args...);

     std::string str(buf);
     delete[] buf;
     return str;
}

// void printUdpHeader(CommunicationHeaderByte* header);
// void printImu(Sim_Msg_IMUData data);
// void printGps(Sim_Msg_GPSData data);
// void printLidar(Sim_Msg_LidarData data);

// UDP Sending Overloads
// int sendUdp(int _port, char* _add, CommunicationHeaderByte* header, Sim_Msg_IMUData data);
// int sendUdp(int _port, char* _add, CommunicationHeaderByte* header, Sim_Msg_GPSData data);
// int sendUdp(int _port, char* _add, CommunicationHeaderByte* header, Sim_Msg_LidarData data);

#endif /* UTILITIES_UTILS_H_ */
