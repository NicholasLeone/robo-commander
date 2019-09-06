#include <pigpiod_if2.h>
#include "communication/i2c.h"

I2C::I2C(int dev, int bus, int add){
     this->_device = dev;
     this->_bus = bus;
     this->_address = add;
     this->_handle = attachPeripheral(I2C_PI, bus, add);
}

I2C::~I2C(){
     int err = i2c_close(_device, _handle);
     if(err < 0) printf("ERROR: Couldnt't close i2c line\n\r");
}

int I2C::attachPeripheral(PERIPHERAL_PROTOCOL peripheral, int channel, int id){
     int comm_handle;

     if(peripheral == I2C_PI){
          comm_handle = i2c_open(_device, channel, id, 0);
     }else{
          printf("ERROR: Communication interface incorrectly set!\r\n");
     }
     return comm_handle;
}

int I2C::close(){
     int err = i2c_close(_device, _handle);
     if(err < 0){
          printf("ERROR: Couldnt't close i2c line\n\r");
          return err;
     }
     return 0;
}

void I2C::set_platform_handle(int device){this->_device = device;}
void I2C::set_address(int add){this->_address = add;}
void I2C::set_bus(int bus){this->_bus = bus;}

int I2C::get_platform_handle(){ return this->_device;}
int I2C::get_communication_handle(){ return this->_handle;}
int I2C::get_address(){ return this->_address;}
int I2C::get_bus(){ return this->_bus;}


/**  Parent Functions:
*    TODO: Make these easily handle different platforms (Pi, Arduino, etc.)
*/
int I2C::write_raw_byte(uint8_t byte, bool verbose){
     int err = i2c_write_byte(_device,_handle, byte);
     if(err < 0){
          printf("ERROR: i2c couldnt't write byte\n\r");
          return err;
     }
     if(verbose) printf("Wrote Value 0x%02X to address 0x%02X\r\n",byte,this->_address);
     return 0;
}

uint8_t I2C::read_raw_byte(uint8_t reg, bool verbose){
     uint8_t data = i2c_read_byte_data(_device,_handle, reg);
     if(data < 0){
          printf("ERROR: i2c couldnt't access register at %d to read\n\r", reg);
          return -2;
     }
     if(verbose) printf("Read Value 0x%02X to address 0x%02X\r\n",data,reg);
     return data;
}

/**  Children Functions:
*         These functions are derived from the parent functions and just provide
*    more compacted functions for ease-of-devel purposes.
*/
int I2C::write_byte(uint8_t reg, uint8_t byte, bool verbose){
     int err = i2c_write_byte_data(_device,_handle, reg, byte);
     if(err < 0){
          printf("ERROR: i2c couldnt't write to register at address %d\n\r",int(reg));
          return err;
     }
     if(verbose) printf("Wrote Value 0x%02X to register 0x%02X on address 0x%02X\r\n",byte,reg,this->_address);
     return 0;
}

int I2C::write_bytes(uint8_t reg, char* buf, bool verbose){
     int numBytes = (sizeof(buf)/sizeof(*buf));
     // cout << "Length of input buffer = " << numBytes << endl;

     // int err = i2c_write_block_data(_device,_handle, add, buf, numBytes);
     int err = i2c_write_i2c_block_data(_device,_handle, reg, buf, numBytes);
     if(verbose) printf("Wrote bytes to register 0x%02X on address 0x%02X\r\n",reg,this->_address);
     return err;
}

int I2C::read_bytes(uint8_t reg, char* data, bool verbose){
     int err = i2c_read_block_data(_device,_handle, reg, data);

     if(err < 0){
          printf("ERROR: Could not read in bytes from register at address '%d'!!!\r\n",reg);
     }else{
          printf("I2C Read Success: %d Bytes read from register %d",err,reg);
     }
     if(verbose) printf("Read Bytes from register 0x%02X on address 0x%02X\r\n",reg,this->_address);
     return err;
}


/**
*         TODO: Placeholder area for target-platform independent functions to
* allow for same execution of functions across all platforms (i.e Pi vs Arduino)
*
*/
int _initI2c(int i2cBus, int address){return 0;}
int _outputPWM(int channel, int val){return 0;}
