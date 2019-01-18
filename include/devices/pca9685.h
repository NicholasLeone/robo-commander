#ifndef PCA9685_H_
#define PCA9685_H_

#include "comms/i2c.h"

#define MODE1 0x00			// Mode  register  1
#define MODE2 0x01			// Mode  register  2
#define SUBADR1 0x02		// I2C-bus subaddress 1
#define SUBADR2 0x03		// I2C-bus subaddress 2
#define SUBADR3 0x04		// I2C-bus subaddress 3
#define ALL_CALL_ADR 0x05     // LED All Call I2C-bus address
#define LED0 0x6			// LED0 start register
#define LED0_ON_L 0x6		// LED0 output and brightness control byte 0
#define LED0_ON_H 0x7		// LED0 output and brightness control byte 1
#define LED0_OFF_L 0x8		// LED0 output and brightness control byte 2
#define LED0_OFF_H 0x9		// LED0 output and brightness control byte 3
#define LED_MULTIPLYER 4	     // For the other 15 channels
#define ALL_LED_ON_L 0xFA     // load all the LEDn_ON registers, byte 0 (turn 0-7 channels on)
#define ALL_LED_ON_H 0xFB	// load all the LEDn_ON registers, byte 1 (turn 8-15 channels on)
#define ALL_LED_OFF_L 0xFC	// load all the LEDn_OFF registers, byte 0 (turn 0-7 channels off)
#define ALL_LED_OFF_H 0xFD	// load all the LEDn_OFF registers, byte 1 (turn 8-15 channels off)
#define PRE_SCALE 0xFE		// prescaler for output frequency

// Bits:
#define PCA9685_RESTART 0x80
#define PCA9685_SLEEP 0x10
#define PCA9685_ALLCALL 0x01
#define PCA9685_INVRT 0x10
#define PCA9685_OUTDRV 0x04

class PCA9685 : public I2C{
private:

     float _frequency;
     float _period_pulsewidth;
     int _dev;
     int _han;

public:

	PCA9685(int dev, int bus, int address);
	~PCA9685();

     /**       Function: Sets the output frequency of the PWM signals from all the channels
     *    @param freq: Desired PWM frequency (40Hz - 1000Hz)
     */
	int setFrequency(int freq);

     /**       Function: Sets the PWM channel dependant on steps on/off
     *    @param channel: Desired PWM channel to manipulate (0 - 15) -> Use -1 for all channels
     *    @param on_val:  Desired step value associated with the time on (0 - 4095)
     *    @param off_val: Desired step value associated with the time off (0 - 4095)
     */
     int setStep(int channel, int on_val, int off_val);

     /**       Function: Sets the PWM channel's step value associated with desired dutycycle response
     *    @param channel : Desired PWM channel to manipulate (0 - 15) -> Use -1 for all channels
     *    @param duty    : Desired duty cycle (0 - 100%)
     */
	int setDutyCycle(int channel, float duty);

     /**       Function: Sets the PWM channel's step value associated with the desired PWM pulsewidth
     *    @param channel : Desired PWM channel to manipulate (0 - 15) -> Use -1 for all channels
     *    @param width: Desired pulsewidth of the PWM signal (limited by the operating frequency)
     */
     int setPulsewidth(int channel, int width);

     /**       Function: Sets all channels' values to zero and closes i2c line */
     void shutdown();
};
#endif
