#ifndef CAMERA_GIMBAL_H_
#define CAMERA_GIMBAL_H_

#include "comms/i2c.h"
#include "devices/pca9685.h"
#include "sensors/bno055.h"
#include "controls/pid.h"

class CameraGimbal : public PID{
private:

     float _frequency;
     float _period_pulsewidth;
     int _dev;
     int _han;

public:
     CameraGimbal();
	CameraGimbal(int dev, int bus, int address);
	virtual ~CameraGimbal();

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
#endif /** CAMERA_GIMBAL_H_ */
