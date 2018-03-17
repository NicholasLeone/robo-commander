#ifndef ENCODER_H_
#define ENCODER_H_

#define ENCODER_MODE_DETENT 0
#define ENCODER_MODE_STEP   1

typedef void (*ENC_CB_t)(int);

typedef struct encoder_t{
     int pinA;
     int pinB;
     int pinIndex;
     int PPR;
     int gearRatio;

     int filter_time;
     int mode;
     ENC_CB_t cb;
     int cb_id_a;
     int cb_id_b;
     int cb_id_index;
     int levA;
     int levB;
     int oldState;
     int step;
} encoder_t;


class Encoder{

private:
     int _pi;
     float _dt;
     static void _cb(int pi, unsigned gpio, unsigned level, uint32_t tick, void* myEnc);
     static void _cbIndex(int pi, unsigned gpio, unsigned level, uint32_t tick, void* myEnc);
public:

     float _speed;
     float avg_speed;
     int moving_avg_samples;
     encoder_t params;

     Encoder(int myPi, int Apin, int Bpin, int indexPin, int pulsePerRev, int encGearRatio, int filterNoise, int mode, ENC_CB_t cb_func);
     ~Encoder();

     int getPosition();
     float getSpeed(int calc_meth);
     void _moving_average(Encoder* tmpEnc, float sample,int* i,float* tmp_sum);

     static void calc_speed(Encoder* tmpEnc);
};



#endif /** ENCODER_H_ */
