#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_
#define PLATFORM_SPECIFIC extern

#include <stdint.h>

enum PERIPHERAL_PROTOCOL {
	I2C,
	UART,
	GPIO,
	PWM,
	SPI,
	BLUETOOTH,
	WIFI,
	BLUETOOTH_LE,
	MAVLINK2_0,
	MAVLINK3_0
};

typedef struct PIN_CONFIG{
     int pinNum;
     int pinLevel;
} PIN_CONFIG;

typedef struct Peripheral{
	PERIPHERAL_PROTOCOL protocol;
	int pin;
}Peripheral;


class Hardware{
protected:
     PERIPHERAL_PROTOCOL communication_protocol;
public:
	virtual int attachPeripheral(PERIPHERAL_PROTOCOL peripheral, int channel, int id) = 0;
};


class Motor : public Hardware{
public:
	// virtual ERROR setWheelVelocity(float vel) = 0;
protected:
	PERIPHERAL_PROTOCOL communication_protocol;
};


#endif /** PERIPHERALS_H_ */
