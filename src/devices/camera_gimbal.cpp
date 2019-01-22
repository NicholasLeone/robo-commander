#include <iostream>
#include "devices/camera_gimbal.h"

CameraGimbal::CameraGimbal(){}

CameraGimbal::~CameraGimbal(){
     delete this->gimbal;
}

int CameraGimbal::_init_sensor(COMMUNICATION_CONFIGURATION comms){
     this->sensor.init(comms.serial.address, comms.serial.baud);
     int err = this->sensor.begin();
     if(err < 0){
          printf("[ERROR] BNO055::begin] ---- %d.\r\n", err);
          return -1;
     }else{
          printf("[SUCCESS] BNO-055 Initialized \r\n\r\n");
          return 1;
     }
     return 0;
}

int CameraGimbal::_init_actuator(COMMUNICATION_CONFIGURATION comms, int channel){
     this->gimbal = new PCA9685(comms.platform, comms.i2c.bus, comms.i2c.address);
     this->actuator_channel = channel;
     return 0;
}

int CameraGimbal::init(COMMUNICATION_CONFIGURATION comms, int channel){
     if(this->_init_actuator(comms, channel) >= 0){
          printf("[SUCCESS] Camera Gimbal Actuator Initialized!\r\n");
     }else{
          printf("[ERROR] Camera Gimbal Actuator Failed to initialized!\r\n");
          return -1;
     }

     if(this->_init_sensor(comms) >= 0){
          printf("[SUCCESS] Camera Gimbal Sensor Initialized!\r\n");
     }else{
          printf("[ERROR] Camera Gimbal Sensor Failed to initialized!\r\n");
          return -2;
     }
     /** Store communnication configuration for posible usage later */
     this->_comms = comms;

     /** PID Initial Configuration */
     this->_params.dt = 0.01;
     this->_params.max_cmd = 1.0;
     this->_params.min_cmd = -1.0;
     this->_params.max_error = 0.3;
     this->_params.min_error = -0.3;
     this->_params.pre_error = 0;
     this->_params.Kp = 1.0;
     this->_params.Ki = 1.0;
     this->_params.Kd = 1.0;

     return 0;
}

void CameraGimbal::goto_neutral_state(){
     this->gimbal->setPulsewidth(actuator_channel,this->_params.null_cmd);
}

void CameraGimbal::updateOnce(){
     float angles[3];
     this->loopCount++;
     /** Update sensor feedback */
     this->sensor.get_euler(&angles[0]);
     float angle = angles[1];
     float normalized = angle / this->max_state;

     /** Update Control Output */
     float command = this->calculate(normalized);
     command = command * (600.0) + this->_params.null_cmd;
     this->gimbal->setPulsewidth(this->actuator_channel,(int)command);
     float error = this->get_integral_error();
     std::cout << "Angle, Controls, Error: " << angle << "		" << command  << "		" << error << std::endl;

     /** Sleep for specified time */
     usleep(this->_params.dt);
}

void CameraGimbal::set_max_state(float value){ this->max_state = value; }
void CameraGimbal::set_min_state(float value){ this->min_state = value; }
void CameraGimbal::set_neutral_state(float value){ this->neutral_state = value; }
