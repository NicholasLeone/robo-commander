#ifndef I2C_H_
#define I2C_H_

#include "base/peripherals.h"

class I2C : public Communication{
private:

public:
     int _dev;
     int _han; // Communication Handle
     virtual int attachPeripheral(PERIPHERAL_PROTOCOL peripheral, int channel, int id);

     I2C(int dev, int bus, int add);
     ~I2C();

     /* Configurable Parameter Read/Write Functions */
     void set_device(int device);
     void set_bus(int bus);
     void set_address(int add);
     int close();

     /* Parent Functions */
     int _write(uint8_t byte);
     uint8_t _read(uint8_t add);

     /** Derived Functions */
     int _write_byte(uint8_t add, uint8_t byte);
     int write(uint8_t add, char* buf);
     int read(uint8_t add, char* data);
};

/* Placeholder functions just to allow deprecated code to build */
int _initI2c(int i2cBus, int address);
int _outputPWM(int channel, int val);

#endif /* I2C_H_ */
