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

typedef void (*void_int_fun)(int);

using namespace std;

int convertSpdRatio2Pulse(float spd_ratio, int max, int min, int neutral);
void LoadInitialVariables(const string &fileName, map<string, float> &variables);
int countData(string s, char delimiter);
int countLines(const string &fileName);
int extract_bit(int inputByte, int bitLocation);
int extract_bits(int inputByte, int msb, int lsb);
float unpackFloat(char* buffer, int *i);
vector<float> parseFloat(string s, string delimiter);
void attach_CtrlC(void_int_fun func2call);
void attach_CtrlZ(void_int_fun func2call);

#endif /* UTILS_H_ */
