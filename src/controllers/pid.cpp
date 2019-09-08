#include "controllers/pid.h"

using namespace std;
using namespace chrono;

/**
* Constructors & De-Constructors
*/
PID::PID(){}

PID::PID(PID_PARAMS params){
     this->_params = params;
     this->_prev_time = high_resolution_clock::now();
     this->_prev_error = 0;
}

PID::~PID(){}

/** ========================
*          Setters
* ========================== */
void PID::set_params(PID_PARAMS params){ this->_params = params; }
void PID::set_p_gain(float gain){ this->_params.Kp = gain; }
void PID::set_i_gain(float gain){ this->_params.Ki = gain; }
void PID::set_d_gain(float gain){ this->_params.Kd = gain; }
void PID::set_dt(float dt){ this->_params.dt = dt; }
void PID::set_target_state(float target){ this->_target = target; }
void PID::set_integral_error(float error){ this->_prev_error = error; }
void PID::set_max_integral_error(float error){ this->_params.max_error = error; }
void PID::set_min_integral_error(float error){ this->_params.min_error = error; }
void PID::set_max_cmd(float value){ this->_params.max_cmd = value; }
void PID::set_min_cmd(float value){ this->_params.min_cmd = value; }
void PID::set_null_cmd(float value){ this->_params.null_cmd = value; }

/** ========================
*          Getters
* ========================== */
PID_PARAMS PID::get_params(){ return this->_params; }
float PID::get_p_gain(){ return this->_params.Kp;}
float PID::get_i_gain(){ return this->_params.Ki;}
float PID::get_d_gain(){ return this->_params.Kd;}
float PID::get_dt(){ return this->_params.dt; }
float PID::get_target_state(){ return this->_target; }
float PID::get_integral_error(){ return this->_prev_error; }
float PID::get_max_integral_error(){ return this->_params.max_error; }
float PID::get_min_integral_error(){ return this->_params.min_error; }
float PID::get_max_cmd(){ return this->_params.max_cmd; }
float PID::get_min_cmd(){ return this->_params.min_cmd; }
float PID::get_null_cmd(){ return this->_params.null_cmd; }

/** ========================
*    Calculate New Command
* ========================== */
float PID::calculate(float current_state){
     float control_output, error;
     float integral = this->_prev_integral;

     // Get dt since last sensor feedback
     high_resolution_clock::time_point now = high_resolution_clock::now();
     duration<float> time_span = duration_cast<duration<float>>(now - this->_prev_time);
     float dt = time_span.count();

     if(dt >= this->_params.dt){
          // Calculate error
          error = this->_target - current_state;
          // Proportional term
          float Pout = this->_params.Kp * error;
          // Integral term
          integral += (this->_params.Ki * error * dt);

          // Limit integral error within a specified bounds
          if(integral > this->_params.max_error) integral = this->_params.max_error;
          else if(integral < this->_params.min_error) integral = this->_params.min_error;

          // Derivative term
          float dInput = (current_state - this->_prev_state) / dt;
          float Dout = this->_params.Kd * dInput;

          // Calculate total output
          control_output = Pout + integral - Dout;
          // Restrict to max/min
          if(control_output > this->_params.max_cmd) control_output = this->_params.max_cmd;
          else if(control_output < this->_params.min_cmd) control_output = this->_params.min_cmd;
     }else{
          error = this->_target - current_state;
          control_output = this->_prev_output;
     }

     // Remember Current Values for next iteration
     this->_prev_time = now;
     this->_prev_output = control_output;
     this->_prev_state = current_state;
     this->_prev_error = error;
	this->_prev_integral = integral;

     return control_output;
}
