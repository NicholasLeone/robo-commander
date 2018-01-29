#ifndef UDP_H_
#define UDP_H_

#include "base/params.h"
#include <vector>

using namespace std;

class UDP{

private:

     int port;
     char sink_ip[100];
     int nFail;
     time_t timeout_begin;
     int flag_wait;

public:
     char buf[1000];

     UDP_PARAMS* config;
     UDP(int port, char* address = NULL);
     ~UDP();

     int _open(UDP_PARAMS* _config, int port, char* address);
     int _close();
     int _write(UDP_PARAMS* _config, char* _buf, int num_bytes, int port, char* address);
     int _read(UDP_PARAMS* _config, char* _buf, int num_bytes);

     char* read(int num_bytes);
     char* readtimeout(int num_bytes);
     int write(char* buf, int num_bytes, char* address, int port);
};



#endif // UDP_H_
