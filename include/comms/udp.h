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

typedef struct CommunicationHeaderByte{
     int32_t header;
     int32_t msg_type;
     int32_t data_type;
     int32_t measurement_type;
     int32_t measurement_length;
}CommunicationHeaderByte;

typedef struct RC_COMMAND_MSG{
     int32_t yaw;
     int32_t pitch;
	int32_t speed;
}RC_COMMAND_MSG;

class UDP{

private:

     int port;
     char sink_ip[100];
     int nFail;
     time_t timeout_begin;
     int flag_wait;
     int flag_verbose;

public:
     char buf[1000];

     UDP_PARAMS* config;
     UDP(int port, char* address = NULL);
     ~UDP();

     void set_verbose(int flag);

     int _open(UDP_PARAMS* _config, int port, char* address);
     int _close();
     int _write(UDP_PARAMS* _config, char* _buf, int num_bytes, int port, char* address);
     int _read(UDP_PARAMS* _config, char* _buf, int num_bytes);

     char* read(int num_bytes);
     char* readtimeout(int num_bytes);
     int write(char* buf, int num_bytes, char* address, int port);

};



#endif // UDP_H_
