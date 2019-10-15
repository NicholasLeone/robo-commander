#include <iostream>
#include <unistd.h>                // For usleep
#include <limits>                  // For infinity
#include "sensors/camera_d415.h"

using namespace std;

CameraD415::CameraD415(){
     bool verbose = true;
     vector<rs2::device> devList = this->get_available_devices(true);
     this->_dev = devList[0];
     bool err = this->reset(_height, _width, _fps, false);
     usleep(5.0 * 1000000);
     // Try initializing camera hardware
     if(!this->start(_height, _width, _fps)){
          // Attempt to initialize hardware with a reset, if unsuccessful initialization
          if(!this->reset(_height, _width, _fps, true)){
               // if unsuccessful initialization after two reset attempts give up
               if(!this->reset(_height, _width, _fps, true)){
                    printf("[ERROR] Could not initialize CameraD415 object!\r\n");
               }
          }
     }
     printf("SUCCESS: CameraD415 initialized!\r\n");
}

CameraD415::CameraD415(int height, int width, int fps){
     bool verbose = true;
     this->_height = height;
     this->_width = width;
     this->_fps = fps;

     vector<rs2::device> devList = this->get_available_devices(true);
     this->_dev = devList[0];
     bool err = this->reset(height, width, fps, false);
     usleep(5.0 * 1000000);
     // Try initializing camera hardware
     if(!this->start(height, width, fps)){
          // Attempt to initialize hardware with a reset, if unsuccessful initialization
          if(!this->reset(height, width, fps, true)){
               // if unsuccessful initialization after two reset attempts give up
               if(!this->reset(height, width, fps, true)){
                    printf("[ERROR] Could not initialize CameraD415 object!\r\n");
               }
          }
     }
     printf("SUCCESS: CameraD415 initialized!\r\n");
}

CameraD415::~CameraD415(){
     this->stop();
}

bool CameraD415::start(int height, int width, int fps){
     try{
          rs2::config cfg;
          cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);
          cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
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
          std::cerr << "[ERROR] CameraD415::start() --- Could not initialize rs2::pipeline_profile. RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
          return false;
     }
     return true;
}

bool CameraD415::stop(){
     this->_pipe.stop();
     return true;
}

bool CameraD415::reset(int height, int width, int fps, bool with_startup){
     if(this->_dev) this->_dev.hardware_reset();
     else{
          printf("[ERROR] CameraD415::reset() --- No device to reset.\r\n");
          return false;
     }
     if(with_startup) return this->start(height, width, fps);
     else return true;
}

void CameraD415::get_intrinsics(bool verbose){
     float pp[2] = {0.0, 0.0};
     float ff[2] = {0.0, 0.0};
     float ss[2] = {0.0, 0.0};

     if(this->_profile){
          rs2::video_stream_profile depth_stream = this->_profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
          rs2_intrinsics intr = depth_stream.get_intrinsics();

          this->_fx = intr.fx;
          this->_fy = intr.fy;
          this->_ppx = intr.ppx;
          this->_ppy = intr.ppy;

          ff[0] = intr.fx; ff[1] = intr.fy;
          pp[0] = intr.ppx; pp[1] = intr.ppy;
          ss[0] = depth_stream.width(); ss[1] = depth_stream.height();
          if(verbose){
               printf("[INFO] CameraD415::get_intrinsics() --- Intrinsic Properties:\r\n");
               printf("\tSize ---------- [w, h]: %.0f, %.0f\r\n",ss[0],ss[1]);
               printf("\tFocal Length -- [X, Y]: %.2f, %.2f\r\n",ff[0],ff[1]);
               printf("\tPrinciple Point [X, Y]: %.2f, %.2f\r\n",pp[0],pp[1]);
          }
     } else{}
}

void CameraD415::get_extrinsics(bool verbose){
     if(this->_profile){
          rs2::stream_profile depth_stream = this->_profile.get_stream(RS2_STREAM_DEPTH);
          rs2::stream_profile rgb_stream = this->_profile.get_stream(RS2_STREAM_COLOR);
          rs2_extrinsics extr = depth_stream.get_extrinsics_to(rgb_stream);
     }
}

float CameraD415::get_baseline(bool verbose){
     if(this->_profile){
          // rs2::stream_profile ir1_stream = this->_profile.get_stream(RS2_STREAM_INFRARED, 1);
          // rs2::stream_profile ir2_stream = this->_profile.get_stream(RS2_STREAM_INFRARED, 2);
          // rs2_extrinsics extr = ir1_stream.get_extrinsics_to(ir2_stream);
          rs2::stream_profile depth_stream = this->_profile.get_stream(RS2_STREAM_DEPTH);
          rs2::stream_profile rgb_stream = this->_profile.get_stream(RS2_STREAM_COLOR);
          rs2_extrinsics extr = depth_stream.get_extrinsics_to(rgb_stream);

          float baseline = extr.translation[0];
          this->_baseline = baseline;
          if(verbose) printf("[INFO] CameraD415::get_baseline() --- Baseline = %f\r\n",baseline);
          return baseline;
     }else{ return -1.0;}
}

float CameraD415::get_depth_scale(bool verbose){
     if(!this->_profile){
          printf("[ERROR] CameraD415::get_depth_scale() --- Camera Profile is not initialized.\r\n");
          return -1.0;
     } else{
          rs2::depth_sensor sensor = this->_profile.get_device().first<rs2::depth_sensor>();
          float scale = sensor.get_depth_scale();
          this->_depth_scale = scale;
          if(verbose) printf("[INFO] CameraD415::get_depth_scale() --- Depth Scale = %f\r\n",scale);
          return scale;
     }
}

cv::Mat CameraD415::get_rgb_image(){
     cv::Mat frame;
     return frame;
}

cv::Mat CameraD415::get_depth_image(){
     cv::Mat frame;
     return frame;
}

cv::Mat CameraD415::convert_to_disparity(const cv::Mat depth, double& conversion_gain){
     cv::Mat dm, disparity8;
     double minVal, maxVal;
     // std::cout << depth.type() << std::endl;
     depth.convertTo(dm, CV_64F);
     cv::Mat tmp = dm*this->_depth_scale;
     cv::Mat mask = cv::Mat(tmp == 0);
     tmp.setTo(1, mask);
     cv::Mat disparity = (this->_fx * this->_baseline) / tmp;

     disparity.setTo(0, mask);
     // disparity.setTo(0, disparity == std::numeric_limits<int>::quiet_NaN());
     // disparity.setTo(0, disparity == std::numeric_limits<int>::infinity());
     minMaxLoc(disparity, &minVal, &maxVal);
     double gain = 256.0 / maxVal;

     disparity.convertTo(disparity8,CV_8U,gain);
     conversion_gain = gain;
     return disparity8;
}

vector<cv::Mat> CameraD415::read(){
     vector<cv::Mat> imgs;

     if(1){
          // printf("[INFO] CameraD415::update_frames() --- Updating frames...\r\n");
          rs2::frameset frames = this->_pipe.wait_for_frames();
          this->_df = frames.first(RS2_STREAM_DEPTH);
          this->_rgbf = frames.first(RS2_STREAM_COLOR);
          cv::Mat rgb(cv::Size(_width, _height), CV_8UC3, (void*)this->_rgbf.get_data(), cv::Mat::AUTO_STEP);
          cv::Mat depth(cv::Size(_width, _height), CV_16UC1, (void*)this->_df.get_data(), cv::Mat::AUTO_STEP);
          imgs.push_back(rgb);
          imgs.push_back(depth);
     }
     return imgs;
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

// void CameraD415::get_available_sensors(){
//      // Go over the device's sensors
//      for (rs2::sensor& sensor : this->_dev.query_sensors()){
//           // Check if the sensor if a depth sensor
//           if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>()){
//                if(verbose) printf("[INFO] CameraD415::get_depth_scale() --- Depth Scale = %f\r\n",dpt.get_depth_scale());
//                return dpt.get_depth_scale();
//           }
//      }
//      throw std::runtime_error("Device does not have a depth sensor");
// }
