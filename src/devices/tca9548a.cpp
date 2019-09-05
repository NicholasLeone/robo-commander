#include "devices/tca9548a.h"

TCA9548A::TCA9548A(int dev, int bus, int address) : I2C(dev,bus,address){
     I2C::_device = dev;
     I2C::_han = attachPeripheral(I2C_p, bus, address);
}

TCA9548A::~TCA9548A(){
     this->close();
}

int TCA9548A::start(){
     return 0;
}

int TCA9548A::close(){
     return I2C::close();
}
