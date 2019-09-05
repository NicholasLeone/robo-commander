#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_
#define PLATFORM_SPECIFIC extern

#include <string>

using namespace std;

#define BUF_SIZE 256

/** SECTION:

     COMMUNICATION INTERFACES PARAMETERS

*/
typedef struct PIN_CONFIG{
     int pinNum;
     int pinLevel;
} PIN_CONFIG;

enum PERIPHERAL_PROTOCOL {
	I2C_PI,
	UART_PI,
	GPIO_PI,
	PWM_PI,
	SPI_PI,
	BLUETOOTH_PI,
	WIFI_PI,
	BLUETOOTH_LE_PI,
};

typedef struct SERIAL_PARAMS{
     char* add; // Address of device, e.g "/dev/ttyACM0"
     unsigned int baud; // Baud Rate
}SERIAL_PARAMS;


typedef struct I2C_PARAMS{
     int add; // address of device
     int bus; // i2c bus of device
}I2C_PARAMS;

typedef struct INTERFACE_PARAMS{
     string cMeth;  // Method to use for Device Communication
     char* host;    //
     char* port;    //

     // Communications
     I2C_PARAMS i2c;
     SERIAL_PARAMS serParams;

     int pi;        // Raspberry Pi Device is On
     int com;       // Communication Handle of device for pigpiod library

}INTERFACE_PARAMS;

// =====================================================
typedef struct SERIAL_CONFIGURATION{
     char* address; // Address of device, e.g "/dev/ttyACM0"
     int handle;    //
     int baud;
}SERIAL_CONFIGURATION;

typedef struct I2C_CONFIGURATION{
     int bus;
     int address;
     int handle;
     char* location;
}I2C_CONFIGURATION;

typedef struct COMMUNICATION_CONFIGURATION{
     int platform;
     SERIAL_CONFIGURATION serial;
     I2C_CONFIGURATION i2c;
}COMMUNICATION_CONFIGURATION;



/** SECTION:

     INTERFACE PARENT CLASSES

*/

class Hardware{
protected:
     PERIPHERAL_PROTOCOL communication_protocol;
public:
	virtual int attachPeripheral(PERIPHERAL_PROTOCOL peripheral, int channel, int id) = 0;
};


class Motor : public Hardware{
protected:
	PERIPHERAL_PROTOCOL communication_protocol;
public:
     // virtual ERROR setWheelVelocity(float vel) = 0;
};

class Communication : public Hardware{
protected:
	PERIPHERAL_PROTOCOL communication_protocol;

protected:
     int _device; // Device Platform Handle (i.e. Rpi3, Arduino, etc.)
     int _handle; // Communication Interface Handle
     int _address;
     int _bus;
     uint8_t _buf[BUF_SIZE];

};

class CommunicationInterface{


};

#endif /** PERIPHERALS_H_ */
