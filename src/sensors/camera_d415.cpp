#include <iostream>
#include "sensors/camera_d415.h"

using namespace std;

CameraD415::CameraD415(){
     bool verbose = true;
     vector<rs2::device> devList = this->get_available_devices(true);
     this->_dev = devList[0];
     if(!this->start()) printf("[ERROR] Could not initialize CameraD415 object!\r\n");
}

CameraD415::~CameraD415(){
     this->_pipe.stop();
}

int CameraD415::init(const std::string& dev){
     printf("SUCCESS: CameraD415 initialized!\r\n");
     return 1;
}

bool CameraD415::start(){
     try{
          rs2::config cfg;
          cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
          cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
          this->_cfg = cfg;
     } catch(const rs2::error & e){
          std::cerr << "[ERROR] CameraD415::start() --- Could not initialize rs2::config. RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
          return false;
     }

     try{
          rs2::pipeline pipe;
          this->_pipe = pipe;
     } catch(const rs2::error & e){
          std::cerr << "[ERROR] CameraD415::start() --- Could not initialize rs2::pipeline. RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
          return false;
     }

     try{
          rs2::pipeline_profile profile = this->_pipe.start(this->_cfg);
          this->_profile = profile;
     } catch(const rs2::error & e){
          std::cerr << "[ERROR] CameraD415::start() --- Could not initialize rs2::pipeline. RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
          return false;
     }
     return true;
}

bool CameraD415::stop(){
     return true;
}

bool CameraD415::reset(){
     if(this->_dev) this->_dev.hardware_reset();
     else{
          printf("[ERROR] CameraD415::reset() --- No device to reset.\r\n");
          return false;
     }
     return this->start();
}

void CameraD415::get_intrinsics(){}
void CameraD415::get_extrinsics(){}

float CameraD415::get_depth_scale(bool verbose){
     if(!this->_profile){
          printf("[ERROR] CameraD415::get_depth_scale() --- Camera Profile is not initialized.\r\n");
          return -1.0;
     }
     // Go over the device's sensors
     for (rs2::sensor& sensor : this->_dev.query_sensors()){
          // Check if the sensor if a depth sensor
          if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>()){
               if(verbose) printf("[INFO] CameraD415::get_depth_scale() --- Depth Scale = %f\r\n",dpt.get_depth_scale());
               return dpt.get_depth_scale();
          }
     }
     throw std::runtime_error("Device does not have a depth sensor");
}

cv::Mat CameraD415::get_rgb_image(){
     cv::Mat frame;
     return frame;
}

cv::Mat CameraD415::get_depth_image(){
     cv::Mat frame;
     return frame;
}

void CameraD415::grab_images(){
}

void CameraD415::update(){}

vector<rs2::device> CameraD415::get_available_devices(bool verbose){
     vector<rs2::device> devs;
     int index = 0;
     rs2::device_list devices = this->_ctx.query_devices();
     for(rs2::device device : devices){
          std::string name = "Unknown Device";
          std::string sn = "########";
          if(device.supports(RS2_CAMERA_INFO_NAME)) name = device.get_info(RS2_CAMERA_INFO_NAME);
          if(device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) sn = std::string("#") + device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
          if(verbose) std::cout << "  " << index++ << " : " << name + " " + sn << std::endl;
          devs.push_back(device);
     }
     return devs;
}
