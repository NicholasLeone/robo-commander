#include <arpa/inet.h>        // for sockaddr_in and inet_addr()
#include <string.h>             // for memset()
#include <unistd.h>           // for close(), usleep
#include <iostream>           // for cout, endl
#include <fcntl.h>

#include "communication/udp.h"

using namespace std;

UDP::UDP(int port, char* address){
     config = new UDP_PARAMS;

     _open(config, port,address);

     memset(buf, 0, sizeof(buf));
     memset(sink_ip,0,sizeof(sink_ip));

     // Added for timeout read
     this->nFail = 0;
     this->timeoutCnt = 0;
     this->timeout_begin = time(0);
     this->timeout_wait = 0;

     this->flag_wait = 0;
     this->flag_verbose = 0;
}


UDP::~UDP(){ _close(); }

int UDP::_open(UDP_PARAMS* _config, int port, char* address){
     int err, flags;

     bzero(&config->myAddr, sizeof(sockaddr_in));

     _config->sock = socket(AF_INET, SOCK_DGRAM, 0);

     if(_config->sock == 0)
          return -1;

     _config->in_port = port;

     if(!address){
          _config->myAddr.sin_addr.s_addr = INADDR_ANY;
     } else{
          _config->myAddr.sin_addr.s_addr = inet_addr(address);
     }
     _config->myAddr.sin_family = AF_INET;
     _config->myAddr.sin_port = htons(_config->in_port);

     flags = fcntl(_config->sock, F_GETFL);
     flags |= O_NONBLOCK;     // UNCOMMENTED BEFORE
     fcntl(_config->sock, F_SETFL, flags);
     //fcntl(socket_fd, F_SETFL, O_NONBLOCK); //set socket to non-blocking
     // clear the set ahead of time
     FD_ZERO(&_config->read_fds);
     FD_ZERO(&_config->write_fds);

     bzero(&(_config->myAddr.sin_zero),8);

     // Bind the socket to port 14555 - necessary to receive packets from qgroundcontrol
     if(-1 == bind(_config->sock, (struct sockaddr *) &_config->myAddr,sizeof(struct sockaddr))){
          perror("error bind failed");
          close(_config->sock);
          return -2;
     }

     FD_SET(_config->sock, &_config->write_fds);
     FD_SET(_config->sock, &_config->read_fds);

     return 0;
}

int UDP::_close(){

     if(config == NULL)
          return -1;

     close(config->sock);

     return 0;
}

int UDP::_write(UDP_PARAMS* _config, char* _buf, int num_bytes, int port, char* address){

     if(_config == NULL)
          return -1;

     if(_buf == NULL)
          return -2;

     if(num_bytes == 0)
          return -3;

     if(!address)
          return -4;

     memset(&_config->remAddr, 0, sizeof(struct sockaddr_in));
     _config->remAddr.sin_family = AF_INET;
     _config->remAddr.sin_addr.s_addr = inet_addr(address);
     _config->remAddr.sin_port = htons(port);

     int n = sendto(_config->sock,(void*)_buf,num_bytes,0,(struct sockaddr *)&_config->remAddr,sizeof(struct sockaddr_in));

     if(n < 0){
          perror("Sending message");
          return -7;
     }

     return n;
}

int UDP::_read(UDP_PARAMS* _config, char* _buf, int num_bytes){
     int n, receive;
     socklen_t len;

     if(_config == NULL)
          return -1;

     if(_buf == NULL)
          return -2;

     if(num_bytes == 0)
          return -3;

     //int receive = select(ifdata->sock + 1, &ifdata->read_fds, NULL, NULL, &ifdata->tv);

     if(receive == -1){
          //printf("select error read\n");
          //return -4;
     } else if(receive == 0){ //timeout
          //printf("timeout read\n");
          //return 0;
     }

     n = recvfrom(_config->sock,(void*)_buf,num_bytes,0,(struct sockaddr *)&_config->myAddr,&(len=sizeof(struct sockaddr_in)));

     if(flag_verbose == 1){
          cout << "Raw UDP Bytes: ";
          for(int i = 0; i < num_bytes; i++){
               cout << (int)buf[i] << ", ";
          }
          cout << endl;
     }

     if(n < -1){
          perror("recvfrom");
          return -6;
     }

     return n;
}

int UDP::write(char* buf, int num_bytes, char* address, int port){
     int num = this->_write(this->config,buf,num_bytes,port,address);
     return num;
}

char* UDP::read(int num_bytes, bool withTimeout, int maxFails){
     char* recvBuf;
     /** Determine whether to do a blind UDP read or one with a timeout */
     if(!withTimeout){
          this->flush();
          recvBuf = &buf[0];
     } else{ recvBuf = this->timeoutRead(num_bytes,maxFails); }
     return recvBuf;
}

char* UDP::timeoutRead(int num_bytes, int maxFails){
     int nBytes;
     char* recvBuf;
     char dummy[32] = {111, 0, 0, 0, -48, -24, -64, 118, 8, 112, -69, 118,\
          -48, -49, -78, 0, 111, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

     nBytes = this->_read(this->config, buf, num_bytes);
     this->bytesRead = nBytes;

     time_t now = time(0);
     int dt = now - this->timeout_begin;

     if(this->timeout_wait == 0){
          if(nBytes < 0){
               this->nFail++;
          }
          if(dt >= 1){
               if(this->nFail >= maxFails){
                    this->timeout_wait = 1;
               }
               this->timeout_begin = time(0);
               this->nFail = 0;
          }
          recvBuf = &buf[0];
     }else{
          if(nBytes > 0){ this->timeout_wait = 0; }
          recvBuf = dummy;
     }
     return recvBuf;
}

void UDP::flush(){ while(_read(this->config, buf, 4096) > 0){} }

void UDP::set_verbose(int flag){ this->flag_verbose = flag; }
