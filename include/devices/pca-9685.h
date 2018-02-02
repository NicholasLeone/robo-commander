#ifndef PCA9685_H_
#define PCA9685_H_

#include "comms/i2c.h"


class PCA9685 : public I2C{
private:
     int _pi;
	void reset();

public:
	PCA9685();
	~PCA9685();

	void setPWMFreq(int freq);
	void setPWM(uint8_t, int, int);
	void setPWM(uint8_t, int);
	int getPWM(uint8_t);

};
#endif
