#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <math.h>

#include <pigpiod_if2.h>
#include "comms/i2c.h"

using namespace std;

int main(int argc, char *argv[]){
     int err;
     int bus = 1;
     int add = 0x70;
     int freq = 50;

     int on = 0;
     int off = 0;

     int pi = pigpio_start(NULL,NULL);
     I2C i2cDev(pi, bus, add);

     // ----------- Initialize ----------------
     // self.set_all_pwm(0, 0)

     i2cDev._write_byte(MODE2, PCA9685_OUTDRV);
     i2cDev._write_byte(MODE1, PCA9685_ALLCALL);
     usleep(0.005 * 1000000);  // wait for oscillator
     uint8_t mode1 = i2cDev._read(MODE1);
     mode1 = mode1 & ~PCA9685_SLEEP;  // wake up (reset sleep)
     i2cDev._write_byte(MODE1, mode1);
     usleep(0.005 * 1000000);  // wait for oscillator

     // ----------- Set Frequency ----------------

     float prescale_val = 25000000.0;             // 25 MHz
     prescale_val /= 4096.0;                      // 12-bit
     prescale_val /= float(freq);                 // Desired Frequency
     prescale_val -= 1.0;

     uint8_t prescaler = uint8_t (floor(prescale_val + 0.5));
     uint8_t oldmode = i2cDev._read(0x00);
     uint8_t newmode = (oldmode & 0x7f) | 0x10; // Sleep

     i2cDev._write_byte(0x00,newmode);
     i2cDev._write_byte(0xfe,prescaler);
     i2cDev._write_byte(0x00, oldmode);
     usleep(0.005 * 1000000);
     i2cDev._write_byte(0x00, oldmode | 0x80);

     // ----------- Set Pulse Step ----------------
     i2cDev._write_byte(ALL_LED_ON_L, on & 0xFF);
     i2cDev._write_byte(ALL_LED_ON_H, on >> 8);
     i2cDev._write_byte(ALL_LED_OFF_L, off & 0xFF);
     i2cDev._write_byte(ALL_LED_OFF_H, off >> 8);

     return 0;
}
