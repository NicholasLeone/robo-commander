#ifndef I2C_H_
#define I2C_H_

#include "base/peripherals.h"

class I2C : public Communication{
private:
     // int _dev; // Device Platform Handle (i.e. Rpi3, Arduino, etc.)
     // int _han; // Communication Interface Handle

public:
     virtual int attachPeripheral(PERIPHERAL_PROTOCOL peripheral, int channel, int id);

     I2C(int dev, int bus, int add);
     ~I2C();

     int close();

     /* Configurable Parameter Read/Write Functions */
     void set_platform_handle(int device);
     void set_address(int add);
     void set_bus(int bus);

     int get_platform_handle();
     int get_communication_handle();
     int get_address();
     int get_bus();

     /* Parent Functions */
     int write_raw_byte(uint8_t byte);
     uint8_t read_raw_byte(uint8_t reg);

     int write_byte(uint8_t reg, uint8_t byte);

     int write_bytes(uint8_t reg, char* buf);
     int read_bytes(uint8_t reg, char* data);
};

/* Placeholder functions just to allow deprecated code to build */
int _initI2c(int i2cBus, int address);
int _outputPWM(int channel, int val);

#endif /* I2C_H_ */
