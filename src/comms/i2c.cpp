#include "comms/i2c.h"

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
