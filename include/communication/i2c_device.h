#ifndef I2C_DEVICE_H_
#define I2C_DEVICE_H_

#include "communication/i2c.h"

class I2CDevice : public I2C{
private:

public:
     virtual int attachPeripheral(PERIPHERAL_PROTOCOL peripheral, int channel, int id);

     I2CDevice(int platform, int bus, int address);
     ~I2CDevice();

     int close();

};

#endif /* I2C_DEVICE_H_ */
