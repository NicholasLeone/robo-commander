#include <iostream>
#include <string.h>
#include "uart.h"

using namespace std;

UartDev::UartDev(const char* add, int baud){
	int _handle = -1;

	_handle = open(add, O_RDWR | O_NOCTTY | O_NONBLOCK);		//Open in non blocking read/write mode
	if(_handle == -1){
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART [%s].  Ensure it is not in use by another application\r\n", add);
	}

	tcgetattr(_handle, &this->_settings);
	this->_settings.c_cflag = (speed_t)baud | CS8 | CLOCAL | CREAD;		//<Set baud rate
	this->_settings.c_iflag = IGNPAR;
	this->_settings.c_oflag = 0;
	this->_settings.c_lflag = 0;

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
	int bytes_written = write(this->_fd,bytes,num_bytes);
	printf("UartDev::write_bytes ---- num_bytes , size, bytes_written: %d, %d, %d\r\n",num_bytes,size,bytes_written);
	return bytes_written;
}

int UartDev::read_byte(char* buffer_out){
	char buffer[2];
	int bytes_read = read(this->_fd, &buffer[0], sizeof(char));
	printf("UartDev::read_byte ---- Byte read: %s\r\n",buffer);
	memcpy(buffer_out,&buffer, sizeof(char));
     return bytes_read;
}

int UartDev::read_bytes(char* buffer_out, int num_bytes){
	char buffer[4096];
	int bytes_read = read(this->_fd, &buffer[0], num_bytes);
	printf("UartDev::read_bytes ---- Bytes read [%d]: ", bytes_read);
	for(int i = 0; i < num_bytes; i++){
		cout << (int)buffer[i] << " (0x" << std::hex << (int)buffer[i] << "), ";
	}
	std::cout << std::endl;
	memcpy(buffer_out,&buffer[0], bytes_read);
     return bytes_read;
}

int UartDev::writer(char* bytes, int num_bytes){
	int size = sizeof(bytes)/sizeof(*bytes);
	int max_trys = 4;
	int trys = 0;
	int bytes_written;
	char _buf[4096];

	this->flush();
	while(trys <= max_trys + 1){
		bytes_written = this->write_bytes(&bytes[0], num_bytes);
		usleep(0.003 * 1000000);
		printf("UartDev::write_bytes ---- num_bytes , size, bytes_written: %d, %d, %d\r\n",num_bytes,size,bytes_written);
		int nBytes = this->bytes_available();
		char resp[nBytes];
		// this->read_bytes(&_buf[0], nBytes);
		this->read_bytes(&resp[0], nBytes);
		uint8_t resp_header = (resp[0]) & 0xFF;
          uint8_t resp_status = (resp[1]) & 0xFF;
		printf("[INFO] Response Received (header, status): %d,\t%d\r\n", (int)resp_header,(int)resp_status);
		bool resp_check = ((resp_header == 0xEE) && (resp_status == 0x01) );
		if(resp_check){
               break;
          }
		trys += 1;
	}

	return bytes_written;
}

int UartDev::bytes_available(){
	int num_bytes = 0;
	ioctl(this->_fd, FIONREAD, &num_bytes);
	printf("UartDev::bytes_available ---- Bytes available: %d\r\n",num_bytes);
	return num_bytes;
}

void UartDev::set_verbosity(bool verbosity){ this->_verbose = verbosity; }
