#include <iostream>
#include "uart.h"

using namespace std;

UartDev::UartDev(const char* add, unsigned int baud){
	int _handle = -1;

	_handle = open(add, O_RDWR | O_NOCTTY | O_NONBLOCK);		//Open in non blocking read/write mode
	if (_handle == -1){
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}

	struct termios options;
	tcgetattr(_handle, &options);
	options.c_cflag = (speed_t)baud | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(_handle, TCIFLUSH);
	tcsetattr(_handle, TCSANOW, &options);
}

UartDev::~UartDev(){
     close(this->_fd);
     cout << "UartDev Closed" << endl;
}

string UartDev::read(){
     char c;
     string result;

     while(1){
          asio::read(port, asio::buffer(&c, 1));
          switch(c){
               case '\r':
                    break;
               case '\n':
                    return result;
               default:
                    result += c;
          }
     }

}

int UartDev::_write(char byte){
     int err = 0;
     // unsigned char tx_buffer[20];
	// unsigned char *p_tx_buffer;

	// p_tx_buffer = &tx_buffer[0];
	// *p_tx_buffer++ = 'H';
	// *p_tx_buffer++ = 'e';
	// *p_tx_buffer++ = 'l';
	// *p_tx_buffer++ = 'l';
	// *p_tx_buffer++ = 'o';

	if(this->_handle != -1){
		// int count = write(this->_handle, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));
		int count = write(this->_handle, byte, 1);
		if(count < 0){
			printf("UART TX error\n");
               err = -2;
		}
	}else{
          err = -1;
     }
     return err;
}

int UartDev::write(char* buf){
     return 0;
}

int UartDev::read(char* data){
     return 0;
}

void UartDev::flush(){
     tcflush(this->_fd, TCIFLUSH);
}
