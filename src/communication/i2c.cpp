#include <pigpiod_if2.h>
#include "communication/i2c.h"

I2C::I2C(int dev, int bus, int add){
     this->_dev = dev;
     this->_han = attachPeripheral(I2C_p, bus, add);
}

I2C::~I2C(){
     int err = i2c_close(_dev, _han);
     if(err < 0)
          printf("ERROR: Couldnt't close i2c line\n\r");
}

int I2C::attachPeripheral(PERIPHERAL_PROTOCOL peripheral, int channel, int id){
     int comm_handle;

     if(peripheral == I2C_p){
          comm_handle = i2c_open(_dev, channel, id, 0);
     }else{
          printf("ERROR: Communication interface incorrectly set!\r\n");
     }
     return comm_handle;
}

void I2C::set_device(int device){this->_dev = device;}
void I2C::set_bus(int bus){this->_bus = bus;}
void I2C::set_address(int add){this->_address = add;}
int I2C::close(){
     int err = i2c_close(_dev, _han);
     if(err < 0){
          printf("ERROR: Couldnt't close i2c line\n\r");
          return err;
     }
     return 0;
}

/**  Parent Functions:
*    TODO: Make these easily handle different platforms (Pi, Arduino, etc.)
*/
int I2C::_write(uint8_t byte){
     int err = i2c_write_byte(_dev,_han, byte);

     if(err < 0){
          printf("ERROR: i2c couldnt't write byte\n\r");
          return -1;
     }

     return err;
}

uint8_t I2C::_read(uint8_t add){

     uint8_t data = i2c_read_byte_data(_dev,_han, add);

     if(data < 0){
          printf("ERROR: i2c couldnt't access register at %d to read\n\r", add);
          return -2;
     }

     return data;
}

/**  Children Functions:
*         These functions are derived from the parent functions and just provide
*    more compacted functions for ease-of-devel purposes.
*/
int I2C::_write_byte(uint8_t add, uint8_t byte){

     int err = i2c_write_byte_data(_dev,_han, add, byte);
     if(err < 0){
          printf("ERROR: i2c couldnt't write to register at address %d\n\r",int(add));
          return -1;
     }
     return 0;
}

int I2C::write(uint8_t add, char* buf){
     int numBytes = (sizeof(buf)/sizeof(*buf));
     // cout << "Length of input buffer = " << numBytes << endl;

     // int err = i2c_write_block_data(_dev,_han, add, buf, numBytes);
     int err = i2c_write_i2c_block_data(_dev,_han, add, buf, numBytes);

     return err;
}

int I2C::read(uint8_t add, char* data){
     int err = i2c_read_block_data(_dev,_han, add, data);

     if(err < 0){
          printf("ERROR: Could not read in bytes from register at address '%d'!!!\r\n",add);
     }else{
          printf("I2C Read Success: %d Bytes read from register %d",err,add);
     }

     return err;
}


/**
*         TODO: Placeholder area for target-platform independent functions to
* allow for same execution of functions across all platforms (i.e Pi vs Arduino)
*
*/
int _initI2c(int i2cBus, int address){return 0;}
int _outputPWM(int channel, int val){return 0;}
