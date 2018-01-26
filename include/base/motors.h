#ifndef MOTORS_H_
#define MOTORS_H_

#include "actuators.h"
#include "../../include/params.h"

typedef struct PIN_CONFIG{
     int pinNum;
     int pinLevel;
} PIN_CONFIG;

class Motor : public Hardware{

public:
	// virtual ERROR setWheelVelocity(float vel) = 0;
protected:
	PERIPHERAL_PROTOCOL communication_protocol;
};


#endif /** MOTORS_H_ */
