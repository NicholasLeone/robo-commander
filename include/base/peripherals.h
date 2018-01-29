#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_
#define PLATFORM_SPECIFIC extern

#include <stdint.h>
#include "params.h"

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
