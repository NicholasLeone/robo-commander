#include <pigpiod_if2.h>
#include "base/params.h"
#include "comms/i2c.h"

I2C::I2C(int dev, int bus, int add){
     this->_device = dev;
     _han = attachPeripheral(I2C_p, bus, add);
}

I2C::~I2C(){
     int err = i2c_close(_device, _han);
     if(err < 0)
          printf("ERROR: Couldnt't close i2c line\n\r");
}

int I2C::attachPeripheral(PERIPHERAL_PROTOCOL peripheral, int channel, int id){
     int comm_handle;

     if(peripheral == I2C_p){
          comm_handle = i2c_open(_device, channel, id, 0);
     }else{
          printf("ERROR: Communication interface incorrectly set!\r\n");
     }
     return comm_handle;
}

// int I2C::set_address(int add);
// int I2C::set_device(int device_handle);
//
// int I2C::_write_byte(uint8_t byte);
// int I2C::_write_bytes(uint8_t* bytes);
// int I2C::write(uint8_t* buf);
//
// uint8_t I2C::_read_byte(int add);
// uint8_t* I2C::_read_bytes(int add);
// uint8_t* I2C::read(int add);


int _initI2c(int i2cBus, int address){

     int err;

//      if(err = PCA9685_config_and_open_i2c(&myConfig, i2cBus, (uint8_t) address, MY_MODE1, MY_MODE2, PCA9685_FUTABAS3004_PWM_PERIOD)){
// #ifdef TEST_DEBUG
//           printf("couldnt config and open i2c: err %d\n", err);
// #endif
//           return -1;
//      }
//
//      if(err = PCA9685_wake(&myConfig)){
// #ifdef TEST_DEBUG
//           printf("couldnt wake i2c: err %d\n", err);
// #endif
//           return -2;
//      }

     return 0;
}

int _outputPWM(int channel, int val){

     int err;
//      myConfig.channels[channel].dutyTime_us = (uint32_t) val;
//
//      if(err = PCA9685_updateChannelRange(0, NUM_CHANNELS-1, &myConfig)){
// #ifdef TEST_DEBUG
//           printf("couldnt update channels: err %d\n", err);
// #endif
//           return -3;
//      }

     return 0;
}
