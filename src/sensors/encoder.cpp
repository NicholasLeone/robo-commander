#include <unistd.h>
#include <iostream>
#include <chrono>
#include <pigpiod_if2.h>

#include "encoder.h"

using namespace std;
using namespace chrono;

static int transits[16] = {
/* 0000 0001 0010 0011 0100 0101 0110 0111 */
      0,  -1,   1,   0,   1,   0,   0,  -1,
/* 1000 1001 1010 1011 1100 1101 1110 1111 */
     -1,   0,   0,   1,   0,   1,  -1,   0
};

Encoder::Encoder(int myPi, int Apin, int Bpin, int indexPin, int pulsePerRev, int encGearRatio, int filterNoise, int mode, ENC_CB_t cb_func){

     this->_pi = myPi;
     params.pinA = Apin;
     params.pinB = Bpin;
     params.pinIndex = indexPin;
     params.PPR = pulsePerRev;
     params.gearRatio = encGearRatio;
     params.mode = mode;
     params.filter_time = filterNoise;
     params.cb = cb_func;
     params.levA = 0;
     params.levB = 0;
     params.step = 0;
     _dt = 0.01;
     moving_avg_samples = 1000;

     // Attach GPIO pins to Raspberry Pi Hardware
     set_mode(_pi, Apin, PI_INPUT);
     set_mode(_pi, Bpin, PI_INPUT);

     // Pull Up/Down is needed for encoder common ground
     set_pull_up_down(_pi, Apin, PI_PUD_UP);
     set_pull_up_down(_pi, Bpin, PI_PUD_UP);

     // Initialize the debounce filter on the Pi
     set_glitch_filter(_pi, Apin, params.filter_time);
     set_glitch_filter(_pi, Bpin, params.filter_time);

     // Find out the initial values of the A and B signals
     params.oldState = (gpio_read(_pi, Apin) << 1) | gpio_read(_pi, Bpin);

     // Monitor the changes in the encoder levels
     params.cb_id_a = callback_ex(_pi, Apin, EITHER_EDGE, _cb, this);
     params.cb_id_b = callback_ex(_pi, Bpin, EITHER_EDGE, _cb, this);

     // Attach Interrupt to 'Index' pin if it is specified
     if(params.pinIndex != -1)
          params.cb_id_index = callback_ex(_pi, indexPin, RISING_EDGE, _cbIndex, this);
}

void Encoder::_cb(int pi, unsigned gpio, unsigned level, uint32_t tick, void* myEnc){
     Encoder* tmpEnc = (Encoder*) myEnc;
     encoder_t* self = &tmpEnc->params;

     int newState, inc, detent;

     if (level != PI_TIMEOUT){
          if (gpio == (unsigned) self->pinA)
               self->levA = level;
          else
               self->levB = level;

          newState = self->levA << 1 | self->levB;

          inc = transits[self->oldState << 2 | newState];

          if (inc){
               self->oldState = newState;

               detent = self->step / 4;

               self->step += inc;

               if (self->cb){
                    if (self->mode == ENCODER_MODE_DETENT){
                         if (detent != (self->step / 4))
                              (self->cb)(self->step / 4);
                    } else
                         (self->cb)(self->step);
               }
          }
     }
}

void Encoder::_cbIndex(int pi, unsigned gpio, unsigned level, uint32_t tick, void* myEnc){
     Encoder* tmpEnc = (Encoder*) myEnc;
     encoder_t* self = &tmpEnc->params;

     int curPulses = self->step;
     cout << "Pulses @ Index Trigger: " << curPulses << endl;
     // If number of pulses is close to what it should be after one revolution then do some more stuff
     //curPulses = curPulses / self->gearRatio
     //float curRev = (float) self->numRev;

}

int Encoder::getPosition(){
    if(params.mode == ENCODER_MODE_DETENT)
         return params.step / 4;
    else
         return params.step;
}

float Encoder::getSpeed(int calc_meth){
     if(calc_meth == 0)
          return _speed;
     else
          return avg_speed;
}

void Encoder::calc_speed(Encoder* tmpEnc){

     int moving_avg_idx = 0;
     float moving_avg_sum = 0;

     float dt_u = (tmpEnc->_dt) * 1000000;
     float CPR = (float) tmpEnc->params.PPR * (float) tmpEnc->params.gearRatio;

     while(1){
          float before = (float) tmpEnc->getPosition();
          high_resolution_clock::time_point t1 = high_resolution_clock::now();
          usleep(dt_u);
          float after = (float) tmpEnc->getPosition();
          high_resolution_clock::time_point t2 = high_resolution_clock::now();

          float dPos = (after - before) / CPR;
          duration<float> time_span = duration_cast<duration<float>>(t2 - t1);
          float dt = time_span.count();

          float spd = dPos / dt;

          tmpEnc->_moving_average(tmpEnc,spd,&moving_avg_idx,&moving_avg_sum);

          tmpEnc->_speed = spd;

          // printf("Dt, Speed:  %.2f |    %.2f\r\n", dt,spd);
     }

}

void Encoder::_moving_average(Encoder* tmpEnc, float sample,int* i,float* tmp_sum){

     *i += 1;
     *tmp_sum += sample;
     int num_samples = tmpEnc->moving_avg_samples;

     if(*i == num_samples){
          float avg = *tmp_sum / *i;
          tmpEnc->avg_speed = avg;
          //cout << "Moving Average: " << avg << endl;
          *tmp_sum = 0;
          *i = 0;
     }
}

/** TODO:
     Ensure proper deletion of memory and any other system critical services
*/
Encoder::~Encoder(){

}
