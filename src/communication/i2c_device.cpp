#include <pigpiod_if2.h>
#include "communication/i2c_device.h"

I2CDevice::I2CDevice(int platform, int bus, int address) : I2C(platform,bus,address){
     this->_device = platform;
     this->_handle = attachPeripheral(I2C_PI, bus, address);
     // I2C::_device = platform;
     // I2C::_handle = attachPeripheral(I2C_p, bus, address);
}

I2CDevice::~I2CDevice(){
     int err = i2c_close(_device, _handle);
     if(err < 0) printf("ERROR: Couldnt't close i2c line\n\r");
}

int I2CDevice::attachPeripheral(PERIPHERAL_PROTOCOL peripheral, int channel, int id){
     int comm_handle;

     if(peripheral == I2C_p){
          comm_handle = i2c_open(_device, channel, id, 0);
     }else{
          printf("ERROR: Communication interface incorrectly set!\r\n");
     }
     return comm_handle;
}

int I2CDevice::close(){
     int err = i2c_close(_device, _handle);
     if(err < 0){
          printf("ERROR: Couldnt't close i2c line\n\r");
          return err;
     }
     return 0;
}
