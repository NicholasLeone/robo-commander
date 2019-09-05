#ifndef TCA9548A_H_
#define TCA9548A_H_

#include "base/peripherals.h"

class TCA9548A : public I2C{
private:

public:
     int _dev; // Base platform handle (i.e. rpi3, arduino, etc.)
     int _han; // Communication Handle
     virtual int attachPeripheral(PERIPHERAL_PROTOCOL peripheral, int channel, int id);

     TCA9548A(int dev, int bus, int add);
     ~TCA9548A();

     int start();
     int close();
};


#endif /* TCA9548A_H_ */
