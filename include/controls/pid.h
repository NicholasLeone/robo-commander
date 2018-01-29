#ifndef PID_H_
#define PID_H_

#include <stdint.h>
#include <chrono>
#include <ctime>

#include "base/params.h"

/** Forum Tips on PID Tuning */
// 1. Start with I=0, D=0 and pick a fixed velocity. Start on the slow end.
// 2. Ramp up P, keep doubling it until the system becomes unstable. Back off (e.g. take half or 1/4 of the unstable P.)
// 3. Experiment with adding a little I or D and see if they improve performance. If not leave at 0.
// 4. Repeat tuning at a few speeds and plot your tuned variables vs. speed.
// 5. Try fitting linear or 2nd order segments to that graph to determine the coefficients at different speeds if needed.

// Kp -  proportional gain
// Ki -  Integral gain
// Kd -  derivative gain
// dt -  loop interval time
// max - maximum value of manipulated variable
// min - minimum value of manipulated variable

using namespace chrono;

class PID {

private:

     PID_PARAMS params;
     high_resolution_clock::time_point prev_time;
     float prev_input;
     float prev_output;
     float prev_error;

public:

     float _integral;

     PID(PID_PARAMS initialParam);
     ~PID();

     //void init(PID_PARAMS initialParam);
     void update(PID_PARAMS updateParam);

     // Returns the manipulated variable given a setpoint and current process value
     float calculate(float setPoint, float curVal);

};

#endif
//
