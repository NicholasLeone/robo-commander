#ifndef PID_H_
#define PID_H_

#include <chrono>

using namespace std;
using namespace chrono;

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

typedef struct PID_PARAMS{
     float dt;
     float Kp;
     float Ki;
     float Kd;
     float pre_error;
     float max_error;
     float min_error;
     float max_cmd;
     float min_cmd;
     float null_cmd;
}PID_PARAMS;

class PID {

private:

     high_resolution_clock::time_point _prev_time;

     float _prev_state;
     float _prev_output;
     float _prev_error;
     float _prev_integral;
     float _target;

public:

     PID();
     PID(PID_PARAMS params);
     virtual ~PID();

     // Returns the manipulated variable given a setpoint and current process value
     virtual float calculate(float current_state);
     virtual void updateOnce(){}

     /** ========================
     *          Setters
     * ========================== */
     virtual void set_params(PID_PARAMS updateParam);
     virtual void set_p_gain(float gain);
     virtual void set_i_gain(float gain);
     virtual void set_d_gain(float gain);
     virtual void set_dt(float dt);
     virtual void set_target_state(float target);
     virtual void set_integral_error(float error);
     virtual void set_max_integral_error(float error);
     virtual void set_min_integral_error(float error);
     virtual void set_max_cmd(float value);
     virtual void set_min_cmd(float value);
     virtual void set_null_cmd(float value);

     /** ========================
     *          Getters
     * ========================== */
     virtual PID_PARAMS get_params();
     virtual float get_p_gain();
     virtual float get_i_gain();
     virtual float get_d_gain();
     virtual float get_dt();
     virtual float get_target_state();
     virtual float get_integral_error();
     virtual float get_max_integral_error();
     virtual float get_min_integral_error();
     virtual float get_max_cmd();
     virtual float get_min_cmd();
     virtual float get_null_cmd();

     /** ========================
     *      Public Variables
     * ========================== */
     PID_PARAMS _params;
     float current_cmd;
};

#endif
