#include <iostream>
#include <vector>
#include <string.h>
#include <algorithm>

#include "utils.h"

using namespace std;
using namespace arma;

int flag_exit = 0;
int glb_count = 0;

static void funExit(int dummy){
     glb_count++;
     printf("Signal Caught: Ctrl + Z %d Times \r\n",glb_count);
     if(glb_count > 5)
          flag_exit = 1;
}

int main(int argc, char *argv[]){


     /** TEST FOR LOADING CONFIG FILE (STRING)
     std::map<std::string, string> variables;
     cout << "Failled\r\n";
     LoadStringVariables("/home/hunter/devel/robo-dev/config/profiles/dualclaw.config", variables);

     string _ser_path = variables["dev"];
     char* ser_path = (char*) _ser_path.c_str();
     int baud = stoi(variables["baud"]);
     float _base_width = stof(variables["base_width"]);
     float _max_speed = stof(variables["max_speed"]);
     float _qpps_per_meter = stof(variables["qpps_per_meter"]);

     // printf("%d %d %d %d %d %d %d %d\r\n",fr_pwm,fr_dir,fl_pwm,fl_dir,rr_pwm,rr_dir,rl_pwm,rl_dir);
     printf("ROBOCLAW SETTINGS: \r\n");
     printf("       Device Address: %s\r\n", ser_path);
     printf("       Claw Baud Rate: %d\r\n", baud);
     printf("       Base Width: %.4f\r\n", _base_width);
     printf("       Max Speed (m/s): %.3f\r\n", _max_speed);
     printf("       QPPS per Meter: %.3f\r\n", _qpps_per_meter);
     printf("\r\n");
     */ //END LOADING CONFIG FILE (STRING)


     /** TEST FOR BIT EXTRACTION
     int byte = 0b01010110;
     int msb = 2;
     int lsb = 1;
     int numBits = msb - lsb + 1;
     int bit = extract_bits(byte,msb,lsb);
     cout << "Extracted [Byte, Bit(s)]: " << bitset<8>(bit << lsb) << ",  " << bitset<3>(bit) << ",  " << bit << endl;
     */ //END BIT EXTRACTION TEST

     /** TEST FOR SIGNAL CALLED FUNCTION
     int count = 0;
     attach_CtrlZ(funExit);

     printf("Press Ctrl+Z to increment the Global Counter\r\n         Waiting for exit conditions (counter > 5)...\r\n");
     while(!flag_exit){

     }
     */ //END SIGNAL CALLED FUNCTION

     /** TEST FOR CSV EXTRACTION
     string csv = "/home/hunter/Documents/Data/Swanson/1m_straight_slow.csv";
     // vector<vector<float>> mat = csv_to_array(csv.c_str());
     vector<vector<float>> entries = csv_extract_columns(csv.c_str());
     vector<float> entry = entries.at(0);

     int n = entry.size();

     cout << "Entry: ";
     for(int i = 0;i<n;i++){
          cout << entry.at(i) << ", ";
     }
     cout << endl;

     */ //END CSV EXTRACTION


     /** TEST FOR CSV EXTRACTION (ARMADILLO)*/
     string csv = "/home/hunter/Documents/Data/Swanson/1m_straight_slow.csv";
     fmat data = csv_to_matrix(csv);
     data.print("Data Extracted from CSV:");

     //END CSV EXTRACTION (ARMADILLO)

     float e[3] = {0, 0, 0};
     fmat d = Ci2b(e);
     d.print("Rotation: ");

     printf("Code Exit\n\r");
     return 0;

}
