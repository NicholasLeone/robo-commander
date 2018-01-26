#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <signal.h>
#include <map>
#include <bitset>
#include "utils.h"

using namespace std;

int flag_exit = 0;
int glb_count = 0;

void funExit(int dummy){
     glb_count++;
     printf("Signal Caught: Ctrl + Z %d Times \r\n",glb_count);
     flag_exit = 1;
}


int main(int argc, char *argv[]){

     // TEST FOR BIT EXTRACTION
     // int byte = 0b01010110;
     // int msb = 2;
     // int lsb = 1;
     // int numBits = msb - lsb + 1;
     // int bit = extract_bits(byte,msb,lsb);
     // cout << "Extracted [Byte, Bit(s)]: " << bitset<8>(bit << lsb) << ",  " << bitset<3>(bit) << ",  " << bit << endl;
     // END BIT EXTRACTION TEST

     // std::function<void(int)> tmpFun = funExit;
     int count = 0;
     attach_CtrlZ(funExit);

     printf("Waiting...\r\n");
     while(!flag_exit){

     }

     printf("Code Exit\n\r");
     return 0;

}
