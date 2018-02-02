#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>
#include "base/peripherals.h"

class I2C : public Communication{
private:
     int _dev;
     int _han; // Communication Handle
     virtual int attachPeripheral(PERIPHERAL_PROTOCOL peripheral, int channel, int id);

public:

     I2C(int dev, int bus, int add);
     ~I2C();

     // int set_address(int add);
     // int set_device(int device_handle);
     //
     // int _write_byte(uint8_t byte);
     // int _write_bytes(uint8_t* bytes);
     // int write(uint8_t* buf);
     //
     // uint8_t _read_byte(int add);
     // uint8_t* _read_bytes(int add);
     // uint8_t* read(int add);

};

int _initI2c(int i2cBus, int address);
int _outputPWM(int channel, int val);

#endif /* I2C_H_ */
