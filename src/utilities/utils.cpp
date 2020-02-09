#include "utilities/utils.h"

#include <fstream>
#include <iostream>
#include <math.h>
#include <signal.h>

#include <armadillo>

using namespace std;

const std::string WHITESPACE = " \n\r\t\f\v";

int convertSpdRatio2Pulse(float spd_ratio, int max, int min, int neutral){
     float dPulse = (max - min)/2;
     float pulse = (float) dPulse * spd_ratio + (float) neutral;
     return (int) pulse;
}

void LoadInitialVariables(const string &fileName, map<string, float> &variables){
     variables.clear();
     char name[256];
     float value;
     int numLines = 0;
     string line,tmpline;
     ifstream myfile(fileName.c_str());

     while(getline(myfile, tmpline)){
          if (sscanf(tmpline.c_str(), "%s = %f", name, &value) == 2){ variables[name] = value; }
          ++numLines;
     }
}
void LoadStringVariables(const string &fileName, map<string, string> &variables){
     variables.clear();

     char name[256];
     char value[1028];
     int numLines = 0;
     string line,tmpline;

     ifstream myfile(fileName.c_str());
     while(getline(myfile, tmpline)){
          if (sscanf(tmpline.c_str(), "%s = %s", name, &value) == 2){ variables[name] = value; }
          ++numLines;
     }
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
     int b[4];
     float out;

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

int countData(string s, char delimiter){
     size_t num = count(s.begin(), s.end(), delimiter);
     num = num + 1;
     return num;
}
int countLines(const string &file){
     int numLines = 0;
     string tmpline;
     ifstream  _file(file);
     while(getline(_file, tmpline)){ ++numLines;}
     return numLines;
}
vector<float> parseFloat(string s, string delimiter){
     vector<float> tmpData;
     int numVar = countData(s, ',');
     tmpData.reserve(numVar);

     int i = 0;
     size_t pos = 0;
     string token;
     while((pos = s.find(delimiter)) != string::npos) {
          token = s.substr(0, pos);
          int tmpLength = token.size();
          char tmpChar[tmpLength];
          token.copy(tmpChar, tmpLength);

          tmpData.push_back(strtof(tmpChar, NULL));
          i++;
          s.erase(0, pos + delimiter.length());
     }
     return tmpData;
}

vector<int> get_csv_size(const string &file){
     ifstream  csv(file);
     string line, field;
     vector<int> dimen;
     dimen.reserve(2);

     dimen.push_back(countLines(file));
     getline(csv,line);
     dimen.push_back(countData(line,','));

     print_vector("Size of .csv: ",dimen);
     return dimen;
}
vector<vector<float>> csv_to_array(const string &file){
     ifstream  csv(file);
     string line, field;
     vector<int> _sz = get_csv_size(file);
     int rows = _sz.at(0);
     int cols = _sz.at(1);

     vector<float> row;
     vector<vector<float>> array;
     while(getline(csv,line)){
          row.clear();
          stringstream ss(line);

          while (getline(ss,field,',')){ row.push_back(strtof(field.c_str(), NULL));}
          array.push_back(row);
     }

     print_vectors("Matrix: ",array);
     return array;
}
vector<vector<float>> csv_extract_columns(const string &file){
     ifstream  csv(file);
     string line, field;

     vector<int> _sz = get_csv_size(file);
     int _rows = _sz.at(0);
     int _cols = _sz.at(1);

     vector<vector<float>> array;
     vector<vector<float>> cols;
     vector<float> tmpRow;
     vector<float> tmpCol;

     while(getline(csv,line)){
          tmpRow.clear();
          stringstream ss(line);

          while (getline(ss,field,',')){ tmpRow.push_back(strtof(field.c_str(), NULL)); }
          array.push_back(tmpRow);
     }

     for(int i = 0;i<_cols;i++){
          tmpCol.clear();
          for(int j = 0;j<_rows;j++){ tmpCol.push_back(array.at(j).at(i)); }
          cols.push_back(tmpCol);
     }

     return cols;
}

arma::fmat csv_to_matrix(const string &file){
     arma::fmat out;
     out.load(file, arma::csv_ascii);
     return out;
}
arma::fmat Ci2b(float angles[3]){
     arma::fmat out;
     float t = angles[0]; // Theta
     float p = angles[1]; // Phi
     float y = angles[2]; // Upsilon

     out  << cos(t)*cos(y) << -sin(y)*cos(p) + cos(y)*sin(p)*sin(t) << sin(p)*sin(y) + cos(y)*sin(t)*sin(p) << arma::endr
          << cos(t)*cos(y) << cos(p)*cos(y) + sin(t)*sin(p)*sin(y) << -sin(p)*cos(y) + sin(t)*sin(p)*sin(y) << arma::endr
          << -sin(t) << cos(t)*sin(p) << cos(p)*cos(t) << arma::endr;

     return out;
}

std::string ltrim(const std::string& s){
     size_t start = s.find_first_not_of(WHITESPACE);
     return (start == std::string::npos) ? "" : s.substr(start);
}
std::string rtrim(const std::string& s){
     size_t end = s.find_last_not_of(WHITESPACE);
     return (end == std::string::npos) ? "" : s.substr(0, end + 1);
}
std::string trim(const std::string& s){ return rtrim(ltrim(s)); }

std::vector<float> extractFloatStringList(std::string inputString, std::string delimiter){
     size_t pos = 0;
     std::string token;
     std::vector<std::string> strVals;
     while ((pos = inputString.find(delimiter)) != std::string::npos) {
          token = trim(inputString.substr(0, pos));
          strVals.push_back(token);
          inputString.erase(0, pos + delimiter.length());
     }
     strVals.push_back(trim(inputString));

     std::vector<float> floats;
     std::string::size_type sz;
     for(std::string tok : strVals){
          float tmpVal = std::stof(tok, &sz);
          floats.push_back(tmpVal);
     }

     return floats;
}

template<typename T> void print_vector(string header, vector<T> vec){
     int n = vec.size();
     cout << header;
     for(int i = 0;i<n;i++){ cout << vec.at(i) << ", ";}
     cout << endl;
}

template<typename T> void print_vectors(string header, vector< vector<T> > vecs){
     int n = vecs.size();
     int m = vecs.at(0).size();
     cout << header;
     for(int i = 0;i<n;i++){
          cout << "     ";
          for(int j = 0;j<m;j++){ cout << vecs.at(i).at(j) << ", "; }
          cout << endl;
     }
     cout << endl;
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
