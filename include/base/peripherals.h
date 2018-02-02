#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_
#define PLATFORM_SPECIFIC extern

#include <stdint.h>
#include "base/params.h"

#define BUF_SIZE 256

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
     int _device;
     int _address;
     int _bus;
     uint8_t _buf[BUF_SIZE];

};


#endif /** PERIPHERALS_H_ */
