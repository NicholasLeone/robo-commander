#include <string.h>
#include <iostream>
#include "communication/uart.h"

using namespace std;

UartDev::UartDev(const char* add, int baud){
	int _handle = -1;

	_handle = open(add, O_RDWR | O_NOCTTY | O_NONBLOCK);		//Open in non blocking read/write mode
	if(_handle == -1){
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART [%s].  Ensure it is not in use by another application\r\n", add);
	}

	tcgetattr(_handle, &this->_settings);
	this->_settings.c_cflag = (speed_t)baud | CS8 | CLOCAL | CREAD;
	this->_settings.c_iflag = IGNPAR;

	/* Set the attributes to the termios structure*/
	if((tcsetattr(_handle,TCSANOW,&this->_settings)) != 0)
		printf("\n[ERROR] Failed to set attributes for UART device [%s]\r\n",add);

	this->_fd = _handle;
	this->_address = add;
	this->flush();
}

UartDev::~UartDev(){
	close(this->_fd);
	cout << "UartDev Closed" << endl;
}

void UartDev::_close(){ close(this->_fd); }

void UartDev::flush(){ tcflush(this->_fd, TCIFLUSH); }

int UartDev::write_byte(char byte){ return write(this->_fd,&byte,1); }

int UartDev::write_bytes(char* bytes, int num_bytes){
	int size = sizeof(bytes)/sizeof(*bytes);
	int bytes_written = write(this->_fd,&bytes[0],num_bytes);
	if(this->_verbose){
		printf("UartDev::write_bytes ---- bytes_written: ");
		for(int i = 0; i < bytes_written; i++){
			cout << (int)bytes[i] << " (0x" << std::hex << (int)bytes[i] << "), ";
		}
		std::cout << std::endl;
	}
	return bytes_written;
}

int UartDev::read_byte(char* buffer_out){
	char buffer[2];
	int bytes_read = read(this->_fd, &buffer, sizeof(char));
	if(this->_verbose) printf("UartDev::read_byte ---- Byte read: %s\r\n",buffer);
	memcpy(buffer_out,buffer, sizeof(char));
     return bytes_read;
}

int UartDev::read_bytes(char* buffer_out, int num_bytes){
	char buffer[num_bytes];
	int bytes_read = read(this->_fd, &buffer, num_bytes);
	memcpy(buffer_out,buffer, bytes_read);

	if(this->_verbose){
		printf("UartDev::read_bytes ---- Bytes read [%d]: ", bytes_read);
		for(int i = 0; i < bytes_read; i++){
			cout << (int)buffer[i] << " (0x" << std::hex << (int)buffer[i] << "), ";
		}
		std::cout << std::endl;

		printf("UartDev::read_bytes ---- Bytes Copied [%d]: ", bytes_read);
		for(int i = 0; i < bytes_read; i++){
			cout << (int)buffer_out[i] << " (0x" << std::hex << (int)buffer_out[i] << "), ";
		}
		std::cout << std::endl;
	}
	return bytes_read;
}

int UartDev::bytes_available(){
	int num_bytes = 0;
	ioctl(this->_fd, FIONREAD, &num_bytes);
	if(this->_verbose) printf("UartDev::bytes_available ---- Bytes available: %d\r\n",num_bytes);
	return num_bytes;
}

void UartDev::set_verbosity(bool verbosity){ this->_verbose = verbosity; }
