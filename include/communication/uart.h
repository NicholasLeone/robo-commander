#ifndef UART_H_
#define UART_H_

#include <stdio.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */
#include <errno.h>   /* ERROR Number Definitions           */
#include <sys/ioctl.h>

using namespace std;

class UartDev{

private:
     struct termios _settings;
     char read_msg_[512];
     int _fd;
     std::string _address;
     bool active;
     bool _verbose = false;
public:

     UartDev(const char* add, int baud);
     ~UartDev();

     void flush();
     int bytes_available();

     void _close();

     int write_byte(char byte);
     int write_bytes(char* bytes, int num_bytes);

     int read_byte(char* buffer_out);
     int read_bytes(char* buffer_out, int num_bytes);

     void set_verbosity(bool verbosity);
};


#endif /* UART_H_ */
