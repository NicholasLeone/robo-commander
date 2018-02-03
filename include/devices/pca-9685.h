#ifndef PCA9685_H_
#define PCA9685_H_

#include "comms/i2c.h"


class PCA9685 : public I2C{
private:
     int _pi;
	void reset();

public:
	PCA9685(int dev, int bus, int address);
	~PCA9685();

	int setPwmFreq(int freq);
	int setPwm(int led, int value);
     int setPwm(int led, int on_val, int off_val);
     int setAllPwm(int on_val, int off_val);
	int getPwm(int led);

};
#endif
