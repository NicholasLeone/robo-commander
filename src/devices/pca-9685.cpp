#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include "PCA9685.h"

PCA9685::PCA9685(int dev, int bus, int address){
     this->_device = dev;
     _han = attachPeripheral(I2C_p, bus, add);

	reset();
	setPwmFreq(1000);
}

PCA9685::~PCA9685(){
	delete i2c;
}

/* Set the frequency of PWM
     @param freq: desired frequency (40Hz - 1000Hz)
*/
int PCA9685::setPwmFreq(int freq){
     int err;
     uint8_t buf[4];

     float prescale_val = 25000000.0;             // 25 MHz
     prescale_val /= 4096.0;                      // 12-bit
     prescale_val /= float(freq);                 // Desired Frequency
	prescale_val -= 1.0;

	// write_byte(MODE1, 0x10);                // Sleep
     // write_byte(PRE_SCALE, prescale_val);    // PWM Frequency Multiplyer
	// write_byte(MODE1, 0x80);                // Restart
	// write_byte(MODE2, 0x04);                // Cascade (default)
     return err;
}

/** Sets PCA9685 Mode back to 0 */
void PCA9685::reset(){
     // write_byte(MODE1, 0x00);
     // write_byte(MODE2, 0x04);
}

/** Set the step of a single PWM channel with a custom on-time
     @param led: PWM channel (1-16)
     @param on_val: Desired PWM step (0 - 4095) value to turn on pulse
     @param off_val: Desired PWM step (0 - 4095) value to turn off pulse
*/
int PCA9685::setPwm(int led, int on_val, int off_val){
     uint8_t buf[4];
	// write_byte(LED0_ON_L + LED_MULTIPLYER * (led - 1), on_value & 0xFF);
	// write_byte(LED0_ON_H + LED_MULTIPLYER * (led - 1), on_value >> 8);
	// write_byte(LED0_OFF_L + LED_MULTIPLYER * (led - 1), off_value & 0xFF);
	// write_byte(LED0_OFF_H + LED_MULTIPLYER * (led - 1), off_value >> 8);
}

/** Easier use function - Set the step of a single PWM channel
@param led: PWM channel (1-16)
@param value: Desired PWM step (0 - 4095) value
*/
int PCA9685::setPwm(int led, int value){
     setPWM(led, 0, value);
}

/** Set the step of all PWM channels with a custom on-time
     @param on_val: Desired PWM step (0 - 4095) value to turn on pulse
     @param off_val: Desired PWM step (0 - 4095) value to turn off pulse
*/
int PCA9685::setAllPwm(int on_val, int off_val){
     uint8_t buf[4];
	// write_byte(LED0_ON_L + LED_MULTIPLYER * (led - 1), on_value & 0xFF);
	// write_byte(LED0_ON_H + LED_MULTIPLYER * (led - 1), on_value >> 8);
	// write_byte(LED0_OFF_L + LED_MULTIPLYER * (led - 1), off_value & 0xFF);
	// write_byte(LED0_OFF_H + LED_MULTIPLYER * (led - 1), off_value >> 8);
}

/** Get current PWM value
     @param led: PWM channel (1-16)
*/
int PCA9685::getPwm(int led){
     uint8_t buf[BUF_SIZE];
	// int ledval = 0;
	// ledval = read_byte(LED0_OFF_H + LED_MULTIPLYER * (led-1));
	// ledval = ledval & 0xf;
	// ledval <<= 8;
	// ledval += read_byte(LED0_OFF_L + LED_MULTIPLYER * (led-1));
	// return ledval;
}
