#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <pigpiod_if2.h>
#include "devices/pca9685.h"
#include "base/definitions.h"

PCA9685::PCA9685(int dev, int bus, int address) : I2C(dev,bus,address){
     I2C::_device = dev;
     I2C::_han = attachPeripheral(I2C_p, bus, address);

     I2C::_write_byte(MODE2, PCA9685_OUTDRV);
     I2C::_write_byte(MODE1, PCA9685_ALLCALL);
     usleep(0.005 * 1000000);                // Wait for oscillator
     uint8_t mode1 = I2C::_read(MODE1);
     mode1 = mode1 & ~PCA9685_SLEEP;         // Wake-up (reset sleep)
     I2C::_write_byte(MODE1, mode1);
     usleep(0.005 * 1000000);                // Wait for oscillator
}

PCA9685::~PCA9685(){}

int PCA9685::setFrequency(int freq){
     int err;

     float prescale_val = 25000000.0;             // 25 MHz Clock
     prescale_val /= 4096.0;                      // 12-bit Resolution
     prescale_val /= float(freq);                 // Desired Frequency
     prescale_val -= 1.0;
     int prescale = int(floor(prescale_val + 0.5));

     if(prescale < 3)
          prescale = 3;
     else if(prescale > 255)
          prescale = 255;

     printf("Prescale Value: %d\n\r", prescale);

     uint8_t oldmode = I2C::_read(MODE1);
     uint8_t newmode = (oldmode & ~PCA9685_SLEEP) | PCA9685_SLEEP;

     err = I2C::_write_byte(MODE1,newmode);                    // Sleep
     err = I2C::_write_byte(PRE_SCALE,prescale);               // PWM Frequency Multiplyer
     err = I2C::_write_byte(MODE1, oldmode);
     usleep(0.005 * 1000000);
     err = I2C::_write_byte(MODE1, oldmode | PCA9685_RESTART); // Restart

     _frequency = (25000000.0 / 4096.0) / (prescale + 1);
     _period_pulsewidth = (1000000.0 / _frequency);
     printf("Frequency, Pulsewidth: %.2f,    %.2f\r\n",_frequency, _period_pulsewidth);

     return err;
}


int PCA9685::setStep(int channel, int on_val, int off_val){return 0;}

int PCA9685::setDutyCycle(int channel, float duty){

     int  on_step, off_step;
     int steps, err;
     int tmpAdd[4];
     uint8_t buf[4];
     uint8_t addOut;

     steps = int(round(duty * (4096.0 / 100.0)));
     printf("# of Steps: %d\n\r", steps);

     // Saturate outputs
     if(steps < 0){
          on_step = 0;
          off_step = 4096;
     }else if(steps > 4095){
          on_step = 4096;
          off_step = 0;
     }else{
          on_step = 0;
          off_step = steps;
     }

     buf[0] = on_step & 0xFF;
     buf[1] = on_step >> 8;
     buf[2] = off_step & 0xFF;
     buf[3] = off_step >> 8;

     // Change desired channel
     if((channel >= 0) && (channel <= 15)){
          tmpAdd[0] = LED0_ON_L + LED_MULTIPLYER * channel;
          tmpAdd[1] = LED0_ON_H + LED_MULTIPLYER * channel;
          tmpAdd[2] = LED0_OFF_L + LED_MULTIPLYER * channel;
          tmpAdd[3] = LED0_OFF_H + LED_MULTIPLYER * channel;
     }else{
          tmpAdd[0] = ALL_LED_ON_L;
          tmpAdd[1] = ALL_LED_ON_H;
          tmpAdd[2] = ALL_LED_OFF_L;
          tmpAdd[3] = ALL_LED_OFF_H;
     }

     for(int i = 0; i < 4; i++){
          err = I2C::_write_byte(tmpAdd[i],buf[i]);
          if(err < 0){
               printf("ERROR: Could not set duty cycle due to error code %d", err);
               return err;
          }
     }

     return 0;
}

int PCA9685::setPulsewidth(int channel, int width){
     float duty = (float(width) / this->_period_pulsewidth) * 100.0;
     setDutyCycle(channel, duty);
}


void PCA9685::shutdown(){
     setDutyCycle(-1,0);
}
