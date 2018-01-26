#include <fstream>
#include <iostream>
#include <signal.h>
#include <bitset>
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <algorithm>
#include <sstream>

#include <boost/asio.hpp>
#include "boost/foreach.hpp"
#include "utils.h"

// #define GTSAM_LIBRARY_INCLUDED

using namespace std;

int convertSpdRatio2Pulse(float spd_ratio, int max, int min, int neutral){

     float dPulse = (max - min)/2;
     float pulse = (float) dPulse * spd_ratio + (float) neutral;

     // cout << "Converted Pulse: " << pulse << endl;
     return (int) pulse;
}

void LoadInitialVariables(const string &fileName, map<string, float> &variables){
    variables.clear();

    char name[256];
    float value;
    int numLines;
    string line,tmpline;

	ifstream myfile(fileName.c_str());

	numLines = 0;

	while(getline(myfile, tmpline)){
		if (sscanf(tmpline.c_str(), "%s = %f", name, &value) == 2){
            variables[name] = value;
		}
		++numLines;
	}
}

int countData(string s, char delimiter){
     size_t num = count(s.begin(), s.end(), delimiter);
     num = num + 1;
     return num;
}

int countLines(const string &fileName){

    int numLines;
    string tmpline;

	ifstream myfile(fileName.c_str());

	numLines = 0;

	while(getline(myfile, tmpline)){
		++numLines;
	}

	return numLines;
}


int extract_bit(int inputByte, int bitLocation){

     int tmpBit = (inputByte >> bitLocation) & ~(~0 << 1);
     return tmpBit;

}

int extract_bits(int inputByte, int msb, int lsb){

     int numBits = msb - lsb + 1;
     int tmpBits = (inputByte >> lsb) & ~(~0 << numBits);

     return tmpBits;
}

float unpackFloat(char* buffer, int *i){
    float out;
    int b[4];

    *i += 4;

     for(int j = 0;j < 4; j++){
		//cout << (int)buffer[j];
          b[j] = (int) buffer[j];
     }

     int16_t B1 = b[0] << 8 | (b[1] & 0xFF);
     int16_t B2 = b[2] << 8 | (b[3] & 0xFF);
     int32_t tmp = B1 << 16 | (B2 & 0xFFFF);

     memcpy(&out, &tmp, sizeof(tmp));
     return out;
}

vector<float> parseFloat(string s, string delimiter){
     size_t pos = 0;
     string token;
     int i = 0;
     int tmpLength;
     int numVar = countData(s, ',');
     vector<float> tmpData(numVar);

     while ((pos = s.find(delimiter)) != string::npos) {
          token = s.substr(0, pos);
          tmpLength = token.size();
          char tmpChar[tmpLength];
          token.copy(tmpChar, tmpLength);

          tmpData.at(i) = strtof(tmpChar, NULL);

          i++;
          s.erase(0, pos + delimiter.length());
     }

     tmpLength = s.size();
     char tmpChar[tmpLength];
     s.copy(tmpChar, tmpLength);

     tmpData.at(i) = strtof(tmpChar, NULL);

     return tmpData;

}

void attach_CtrlC(void_int_fun func2call){
     struct sigaction sigIntHandler;
     sigIntHandler.sa_handler = func2call;
     sigemptyset(&sigIntHandler.sa_mask);
     sigIntHandler.sa_flags = 0;
     sigaction(SIGINT, &sigIntHandler, NULL);

}

void attach_CtrlZ(void_int_fun func2call){

     struct sigaction sigUpHandler;
     sigUpHandler.sa_handler = func2call;
     sigemptyset(&sigUpHandler.sa_mask);
     sigUpHandler.sa_flags = 0;
     sigaction(SIGTSTP, &sigUpHandler, NULL);
}


#ifdef GTSAM_LIBRARY_INCLUDED
#define foreach BOOST_FOREACH

void writeResults(Values &results, string outputFile){
     ofstream resultFile(outputFile.c_str());

     Values::ConstFiltered<Pose2> result_poses = results.filter<Pose2>();
     foreach (const Values::ConstFiltered<Pose2>::KeyValuePair& key_value, result_poses){
          Pose2 p = key_value.value;
          string k = Symbol(key_value.key);
          resultFile << k << " " << p.x() << " " << p.y() << " " << p.theta() << endl;
     }
}
#endif
