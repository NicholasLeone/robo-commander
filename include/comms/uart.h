#ifndef UART_H_
#define UART_H_

#include <stdio.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */
#include <errno.h>   /* ERROR Number Definitions           */

using namespace std;

class UartDev{

private:
     char read_msg_[512];
     int _fd;
     struct termios _settings;
     int _write(char byte);
     char _read();

public:
     bool active;

     UartDev(const char* add, int baud);
     ~UartDev();

     int read(char* data);
     int write(char* buf);
     void flush();

};


#endif /* UART_H_ */
