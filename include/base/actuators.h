#ifndef ACTUATORS_H_
#define ACTUATORS_H_

#define PLATFORM_SPECIFIC extern
#include <stdint.h>
#include "params.h"

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

typedef struct Peripheral{
	PERIPHERAL_PROTOCOL protocol;
	int pin;
}Peripheral;

/******************************************************************************
Class
******************************************************************************/

class Hardware{
protected:
     PERIPHERAL_PROTOCOL communication_protocol;
public:
	virtual int attachPeripheral(PERIPHERAL_PROTOCOL peripheral, int channel, int id) = 0;
};


/******************************************************************************
TODO: Define better
******************************************************************************/
class Motor : public Hardware{
public:
	// virtual ERROR setWheelVelocity(float vel) = 0;
protected:
	PERIPHERAL_PROTOCOL communication_protocol;
};


#endif /** ACTUATORS_H_ */
