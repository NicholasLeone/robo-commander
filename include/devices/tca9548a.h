#ifndef TCA9548A_H_
#define TCA9548A_H_

#include "communication/i2c.h"

class TCA9548A : public I2C{
private:
     int current_channel;
public:

     TCA9548A(int dev, int bus, int address = 0x70);
     ~TCA9548A();

     int get_selected_channel();
     int select_channel(uint8_t channel, bool verbose = false, bool debug = false);
     void scan_bus(I2C* targetDev, bool verbose = false);
};

#endif /* TCA9548A_H_ */
