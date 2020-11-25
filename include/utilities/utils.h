#ifndef ROBOCOMMANDER_UTILITIES_UTILS_H_
#define ROBOCOMMANDER_UTILITIES_UTILS_H_

#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <assert.h>
#include <algorithm>
#include <iterator>
#include <stdint.h>      // For uintX_T types

#include <armadillo>
#include "base/definitions.h"
#include "yaml-cpp/yaml.h"

using namespace std;

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

template<typename T>
std::string vector_str(vector<T> vec, std::string delimiter){
     std::string out;
     std::ostringstream vts;
     if(!vec.empty()){
          std::copy(vec.begin(), vec.end()-1, std::ostream_iterator<T>(vts, delimiter.c_str()));
          vts << vec.back();
          out = vts.str();
     }
     return out;
}

uint64_t get_sys_time_micro();
float unwrap_angle(float angle);

std::string base64_encode(unsigned char const* , unsigned int len);
std::string base64_decode(std::string const& s);

// float convertRadians2Degrees(float angle);
// float convertDegrees2Radians(float angle);
int convertSpdRatio2Pulse(float spd_ratio, int max, int min, int neutral);

std::string base64_encode(unsigned char const* , unsigned int len);
std::string base64_decode(std::string const& s);
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

arma::fmat csv_to_matrix(const string &file);
arma::fmat Ci2b(float angles[3]);

std::string ltrim(const std::string& s);
std::string rtrim(const std::string& s);
std::string trim(const std::string& s);
std::vector<float> extractFloatStringList(std::string inputString, std::string delimiter);

template<typename T> void print_vector(string header, vector<T> vec);
template<typename T> void print_vectors(string header, vector< vector<T> > vecs);

void attach_CtrlC(void_int_fun func2call);
void attach_CtrlZ(void_int_fun func2call);

#endif /* ROBOCOMMANDER_UTILITIES_UTILS_H_ */
