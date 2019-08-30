#ifndef UDP_H_
#define UDP_H_

#include <netdb.h>

using namespace std;

typedef struct UDP_PARAMS{
     int fd;
     int sock;
     int in_port;
     int dev_address;
     int out_port;
     char* file;
     socklen_t sock_len;
     struct sockaddr_in remAddr;
     struct sockaddr_in myAddr;
     struct hostent * hp;
     fd_set read_fds;
     fd_set write_fds;
     struct timeval tv;
}UDP_PARAMS;

class UDP{
private:

     int port;
     char sink_ip[100];
     int nFail;

     time_t timeout_begin;

     int flag_wait;
     int flag_verbose;
     int timeoutCnt;
public:
     char buf[4096];
     int bytesRead;
     int timeout_wait;

     UDP_PARAMS* config;
     UDP(int port, char* address = NULL);
     ~UDP();

     int _open(UDP_PARAMS* _config, int port, char* address);
     int _close();
     int _write(UDP_PARAMS* _config, char* _buf, int num_bytes, int port, char* address);
     int _read(UDP_PARAMS* _config, char* _buf, int num_bytes);

     // char* read(int num_bytes, int maxFails = 15);
     char* read(int num_bytes, bool withTimeout = true, int maxFails = 15);
     char* timeoutRead(int num_bytes, int maxFails = 15);
     int write(char* buf, int num_bytes, char* address, int port);
     void flush();
     void set_verbose(int flag);
};



#endif // UDP_H_
