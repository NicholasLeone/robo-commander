#include <cmath>
#include "pid.h"

using namespace std;
using namespace chrono;

/**
* Constructors & De-Constructors
*/

PID::PID(PID_PARAMS initialParams){
     this->params = initialParams;
     prev_time = high_resolution_clock::now();
     prev_error = 0;
     _integral = 0;
}

PID::~PID(){}

/**
* Functions
*/

// void PID::init(PID_PARAMS initialParams){
//      this->params = initialParams;
// }

void PID::update(PID_PARAMS updateParam){
     this->params = updateParam;
}

float PID::calculate(float setPoint, float curVal){

     float _Kp = this->params.Kp;
     float _Ki = this->params.Ki;
     float _Kd = this->params.Kd;
     float dt = this->params.dt;
     float _max = this->params.max;
     float _min = this->params.min;

     high_resolution_clock::time_point now = high_resolution_clock::now();
     duration<float> time_span = duration_cast<duration<float>>(now - prev_time);
     float _dt = time_span.count();

     if(_dt >= dt){

          // Calculate error
          float error = setPoint - curVal;

          // Proportional term
          float Pout = _Kp * error;

          // Integral term
          _integral += (_Ki * error * _dt);
          if(_integral > _max) _integral = _max;
          else if(_integral < _min) _integral = _min;

          // Derivative term
          float dInput = (curVal - prev_input) / _dt;
          float Dout = _Kd * dInput;

          // Calculate total output
          float output = Pout + _integral - Dout;

          // Restrict to max/min
          if(output > _max) output = _max;
          else if(output < _min) output = _min;

          // Prevent Motor from executing any commands in the direction opposite of the desired direction of motion
          if(setPoint > 0) output = fabs(output);
          else if(setPoint < 0) output = -1 * fabs(output);

          // Remember Current Values for next iteration
          prev_time = now;
          prev_output = output;
          prev_input = curVal;

          return output;
     }
     return prev_output;
}
