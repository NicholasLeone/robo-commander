#include <iostream>
#include <unistd.h>                // For usleep
#include <limits>                  // For infinity

#include "sensors/camera_d415.h"
#include "utilities/image_utils.h"

#define STARTUP_DELAY_SEC 3.0
#define D415_MAX_DEPTH_M 10.0
#define D415_MIN_DEPTH_M 0.1

using namespace std;
using namespace chrono;

filter_options::filter_options(const std::string name, rs2::filter& flt) :
    filter_name(name), filter(flt), is_enabled(true){}

filter_options::filter_options(filter_options&& other) :
    filter_name(std::move(other.filter_name)), filter(other.filter),
    is_enabled(other.is_enabled.load()){}

CameraD415::CameraD415(bool show_options) : _cam_thread(), _stopped(false),
     _do_align(false), _do_processing(false), _thread_started(false), _debug_timings(false), _depth2disparity(true), _disparity2depth(false),
     _proc_queue(MAX_QUEUE), _raw_queue(MAX_QUEUE), _disparity_queue(MAX_QUEUE)
{
     int err;
     bool verbose = true;

     vector<rs2::device> devList = this->get_available_devices(show_options, true);
     this->_dev = devList[0];
     this->_device_name = this->_dev.get_info(RS2_CAMERA_INFO_NAME);

     RS_STREAM_CFG rgb_cfg = {RS2_STREAM_COLOR, _cwidth, _cheight, RS2_FORMAT_BGR8, _cfps};
     RS_STREAM_CFG depth_cfg = {RS2_STREAM_DEPTH, _dwidth, _dheight, RS2_FORMAT_Z16, _dfps};
     std::vector<RS_STREAM_CFG> cfgs = {rgb_cfg, depth_cfg};

     if(this->start_streams(cfgs)) printf("SUCCESS: D415 camera streams initialized!\r\n");
     else exit(0);

     this->_align = new rs2::align(RS2_STREAM_COLOR);
     this->_Dmat = cv::Mat::zeros(5, 1, CV_64F);

     err = this->get_intrinsics(RS2_STREAM_DEPTH,&this->_Kdepth, &this->_Pdepth,true);
     this->_fxd = _Kdepth.at<double>(0);
     this->_fyd = _Kdepth.at<double>(4);
     this->_ppxd = _Kdepth.at<double>(2);
     this->_ppyd = _Kdepth.at<double>(5);

     err = this->get_intrinsics(RS2_STREAM_COLOR,&this->_Krgb, &this->_Prgb);
     this->_fxc = _Krgb.at<double>(0);
     this->_fyc = _Krgb.at<double>(4);
     this->_ppxc = _Krgb.at<double>(2);
     this->_ppyc = _Krgb.at<double>(5);

     float dscale = this->get_depth_scale(verbose);
     float baseline = this->get_baseline(verbose);
     this->_filters = this->get_default_filters();
     printf("[INFO] CameraD415::CameraD415() --- Successfully created.\r\n");
}

CameraD415::CameraD415(int rgb_fps, int rgb_resolution[2], int depth_fps, int depth_resolution[2],
     bool show_options) : _cam_thread(), _stopped(false), _do_align(false), _debug_timings(false),
     _do_processing(false), _thread_started(false), _depth2disparity(true), _disparity2depth(false),
     _proc_queue(MAX_QUEUE), _raw_queue(MAX_QUEUE), _disparity_queue(MAX_QUEUE)
{
     int err;
     bool verbose = true;

     vector<rs2::device> devList = this->get_available_devices(show_options, true);
     this->_dev = devList[0];
     this->_device_name = this->_dev.get_info(RS2_CAMERA_INFO_NAME);

     this->_cfps = rgb_fps;
     this->_cwidth = rgb_resolution[0];
     this->_cheight = rgb_resolution[1];
     this->_dfps = depth_fps;
     this->_dwidth = depth_resolution[0];
     this->_dheight = depth_resolution[1];

     RS_STREAM_CFG rgb_cfg = {RS2_STREAM_COLOR, _cwidth, _cheight, RS2_FORMAT_BGR8, _cfps};
     RS_STREAM_CFG depth_cfg = {RS2_STREAM_DEPTH, _dwidth, _dheight, RS2_FORMAT_Z16, _dfps};
     std::vector<RS_STREAM_CFG> cfgs = {rgb_cfg, depth_cfg};

     if(this->start_streams(cfgs)) printf("SUCCESS: D415 camera streams initialized!\r\n");
     else exit(0);

     this->_align = new rs2::align(RS2_STREAM_COLOR);
     this->_Dmat = cv::Mat::zeros(5, 1, CV_64F);

     err = this->get_intrinsics(RS2_STREAM_DEPTH,&this->_Kdepth, &this->_Pdepth, true);
     this->_fxd = _Kdepth.at<double>(0);
     this->_fyd = _Kdepth.at<double>(4);
     this->_ppxd = _Kdepth.at<double>(2);
     this->_ppyd = _Kdepth.at<double>(5);

     err = this->get_intrinsics(RS2_STREAM_COLOR,&this->_Krgb, &this->_Prgb);
     this->_fxc = _Krgb.at<double>(0);
     this->_fyc = _Krgb.at<double>(4);
     this->_ppxc = _Krgb.at<double>(2);
     this->_ppyc = _Krgb.at<double>(5);

     float dscale = this->get_depth_scale(verbose);
     float baseline = this->get_baseline(verbose);
     this->_filters = this->get_default_filters();
     printf("[INFO] CameraD415::CameraD415() --- Successfully created.\r\n");
}

CameraD415::~CameraD415(){
     this->stop();
     delete this->_align;
}

/*
██ ███    ██ ██ ████████ ██  █████  ██      ██ ███████ ███████
██ ████   ██ ██    ██    ██ ██   ██ ██      ██    ███  ██
██ ██ ██  ██ ██    ██    ██ ███████ ██      ██   ███   █████
██ ██  ██ ██ ██    ██    ██ ██   ██ ██      ██  ███    ██
██ ██   ████ ██    ██    ██ ██   ██ ███████ ██ ███████ ███████
*/

bool CameraD415::stop(){
     this->_pipe.stop();
     this->_stopped = true;
     if(this->_thread_started){
          if(this->_cam_thread.joinable()){ this->_cam_thread.join(); }
     }
     return true;
}
void CameraD415::start_thread(){
     this->_thread_started = true;
     _cam_thread = std::thread(&CameraD415::processingThread,this);
}
bool CameraD415::start_streams(std::vector<RS_STREAM_CFG> stream_cfgs){
     bool err = this->reset(stream_cfgs, false);
     usleep(STARTUP_DELAY_SEC * 1000000);
     // Try initializing camera hardware
     if(!this->hardware_startup(stream_cfgs)){
          // Attempt to initialize hardware with a reset, if unsuccessful initialization
          if(!this->reset(stream_cfgs, true)){
               // if unsuccessful initialization after two reset attempts give up
               if(!this->reset(stream_cfgs, true)){
                    printf("[ERROR] Could not initialize CameraD415 object!\r\n");
                    return false;
               }
          }
     }
     return true;
}
bool CameraD415::sensors_startup(std::vector<RS_STREAM_CFG> stream_cfgs){
     try{
          rs2::config cfg;
          for(std::vector<RS_STREAM_CFG>::iterator it = stream_cfgs.begin(); it != stream_cfgs.end(); ++it){
               cfg.enable_stream((*it).stream_type, (*it).width, (*it).height, (*it).format, (*it).fps);
          }
          this->_cfg = cfg;
     } catch(const rs2::error & e){
          std::cerr << "[ERROR] CameraD415::sensors_startup() --- Could not initialize rs2::config. RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
          return false;
     }
     return true;
}
bool CameraD415::hardware_startup(std::vector<RS_STREAM_CFG> stream_cfgs){
     bool success = this->sensors_startup(stream_cfgs);
     if(!success) return false;

     try{
          rs2::pipeline pipe;
          this->_pipe = pipe;
     } catch(const rs2::error & e){
          std::cerr << "[ERROR] CameraD415::hardware_startup() --- Could not initialize rs2::pipeline. RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
          return false;
     }

     try{
          rs2::pipeline_profile profile = this->_pipe.start(this->_cfg);
          this->_profile = profile;
     } catch(const rs2::error & e){
          std::cerr << "[ERROR] CameraD415::hardware_startup() --- Could not initialize rs2::pipeline_profile. RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
          return false;
     }
     return true;
}
bool CameraD415::reset(std::vector<RS_STREAM_CFG> stream_cfgs, bool with_startup){
     if(this->_dev) this->_dev.hardware_reset();
     else{
          printf("[ERROR] CameraD415::reset() --- No device to reset.\r\n");
          return false;
     }
     if(with_startup) return this->start_streams(stream_cfgs);
     else return true;
}

/*
██ ███    ███  ██████      ██████   █████  ███████ ██  ██████ ███████
██ ████  ████ ██           ██   ██ ██   ██ ██      ██ ██      ██
██ ██ ████ ██ ██   ███     ██████  ███████ ███████ ██ ██      ███████
██ ██  ██  ██ ██    ██     ██   ██ ██   ██      ██ ██ ██           ██
██ ██      ██  ██████      ██████  ██   ██ ███████ ██  ██████ ███████
*/

rs2::frame CameraD415::get_rgb_frame(bool flag_aligned){
     rs2::frame output;
     if((flag_aligned) && (this->_aligned_frames)) output = this->_aligned_frames.first(RS2_STREAM_COLOR);
     else if(this->_frames) output = this->_frames.first(RS2_STREAM_COLOR);
     return output;
}
rs2::frame CameraD415::get_depth_frame(bool flag_aligned){
     rs2::frame output;
     if((flag_aligned) && (this->_aligned_frames)) output = this->_aligned_frames.first(RS2_STREAM_DEPTH);
     else if(this->_frames) output = this->_frames.first(RS2_STREAM_DEPTH);
     return output;
}
int CameraD415::_get_rgb_image(rs2::frame frame, cv::Mat* image){
     if(frame){
          this->_color_frame = frame;
          cv::Mat tmp(cv::Size(_cwidth, _cheight), CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
          *image = tmp;
          return 0;
     } else{
          printf("[WARN] CameraD415::get_rgb_image() ---- Retrieved frame is empty.\r\n");
          cv::Mat tmp = cv::Mat::zeros(_cwidth, _cheight, CV_8UC3);
          *image = tmp;
          return -1;
     }
}
int CameraD415::get_rgb_image(cv::Mat* image, bool flag_aligned){
     rs2::frame tmpFrame = this->get_rgb_frame(flag_aligned);
     if(tmpFrame){
          this->_color_frame = tmpFrame;
          cv::Mat tmp(cv::Size(_cwidth, _cheight), CV_8UC3, (void*)tmpFrame.get_data(), cv::Mat::AUTO_STEP);
          *image = tmp;
          return 0;
     } else{
          printf("[WARN] CameraD415::get_rgb_image() ---- Retrieved frame is empty.\r\n");
          cv::Mat tmp = cv::Mat::zeros(_cwidth, _cheight, CV_8UC3);
          *image = tmp;
          return -1;
     }
}
int CameraD415::_get_depth_image(rs2::frame frame, cv::Mat* image, bool flag_processed){
     int err = 0;
     if(frame){
          this->_depth_frame = frame;
          if(flag_processed){
               rs2::frame tmpFrame;
               err = this->process_depth_frame(frame, &tmpFrame);
               cv::Mat tmp(cv::Size(_dwidth, _dheight), CV_16UC1, (void*)tmpFrame.get_data(), cv::Mat::AUTO_STEP);
               *image = tmp;
          } else{
               cv::Mat tmp(cv::Size(_dwidth, _dheight), CV_16UC1, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
               *image = tmp;
          }
          return 0;
     } else{
          // printf("[WARN] CameraD415::get_depth_image() ---- Retrieved frame is empty.\r\n");
          cv::Mat tmp = cv::Mat::zeros(_dwidth, _dheight, CV_16UC1);
          *image = tmp;
          return -1;
     }
}
int CameraD415::get_depth_image(cv::Mat* image, bool flag_aligned, bool flag_processed){
     int err = 0;
     rs2::frame tmpFrame = this->get_depth_frame(flag_aligned);
     if(tmpFrame){
          cv::Mat tmp;
          err = this->_get_depth_image(tmpFrame, &tmp, flag_processed);
          *image = tmp;
          return err;
     } else{
          printf("[WARN] CameraD415::get_depth_image() ---- Retrieved frame is empty.\r\n");
          cv::Mat tmpNull = cv::Mat::zeros(_dwidth, _dheight, CV_16UC1);
          *image = tmpNull;
          return -1;
     }
     return err;
}
int CameraD415::read(cv::Mat* rgb, cv::Mat* depth, cv::Mat* processed, bool flag_aligned, bool flag_processed){
     int errProc = -1;
     cv::Mat _rgb, _depth, _processed;
     // printf("[INFO] CameraD415::update_frames() --- Updating frames...\r\n");
     this->_frames = this->_pipe.wait_for_frames();
     if(flag_aligned) this->_aligned_frames = this->_align->process(this->_frames);

     int errRgb = this->get_rgb_image(&_rgb, flag_aligned);
     if(errRgb >= 0) this->_nRgbFrames++;
     *rgb = _rgb;

     int errDepth = this->get_depth_image(&_depth, flag_aligned);
     if(errDepth >= 0) this->_nDepthFrames++;
     *depth = _depth;

     if(flag_processed){
          errProc = this->get_depth_image(&_processed, flag_aligned, flag_processed);
          if(errProc > 0) this->_nProcFrames++;
          *processed = _processed;
     }

     int err = errRgb + errDepth;
     if(err < 0) printf("[WARN] CameraD415::read() ---- One or more of the retrieved images are empty.\r\n");
     printf("[INFO] CameraD415::read() ---- nRGB = %d, nDepth = %d, nProcessed = %d.\r\n", this->_nRgbFrames, this->_nDepthFrames, this->_nProcFrames);
     return err;
}

/*
██████  ██████   ██████   ██████ ███████ ███████ ███████
██   ██ ██   ██ ██    ██ ██      ██      ██      ██
██████  ██████  ██    ██ ██      █████   ███████ ███████
██      ██   ██ ██    ██ ██      ██           ██      ██
██      ██   ██  ██████   ██████ ███████ ███████ ███████
*/

int CameraD415::process_depth_frame(rs2::frame frame, rs2::frame* processed){
     int err = 0;
     rs2::frame _processed = frame;
     if(this->_filters.size() > 0){
          int i = 0;
          for(std::vector<rs2::filter>::iterator it = this->_filters.begin(); it != this->_filters.end(); ++it){
               _processed = _processed.apply_filter((*it));
          }
     } else  err = -1;

     *processed = _processed;
     return err;
}
int CameraD415::process_frames(rs2::frameset frames, rs2::frameset* processed){
     int err = 0;
     rs2::frameset _processed = frames;
     if(this->_filters.size() > 0){
          int i = 0;
          for(std::vector<rs2::filter>::iterator it = this->_filters.begin(); it != this->_filters.end(); ++it){
               _processed = _processed.apply_filter((*it));
          }
     } else  err = -1;

     *processed = _processed;
     return err;
}

/*
 ██████  ██████  ███    ██ ██    ██ ███████ ██████  ████████
██      ██    ██ ████   ██ ██    ██ ██      ██   ██    ██
██      ██    ██ ██ ██  ██ ██    ██ █████   ██████     ██
██      ██    ██ ██  ██ ██  ██  ██  ██      ██   ██    ██
 ██████  ██████  ██   ████   ████   ███████ ██   ██    ██
*/

cv::Mat CameraD415::convert_to_disparity(const cv::Mat depth, double* conversion_gain, double* conversion_offset){
     // float trueMaxDisparity = (this->_fxd * this->_baseline)/((float) D415_MAX_DEPTH_M);
     // float trueMinDisparity = (this->_fxd * this->_baseline)/((float) D415_MIN_DEPTH_M);
     // bool use_test = true;
     bool use_test = false;
     bool debug = false;
     cv::Mat tmp, disparity, disparity8;
     double min, maxMeter, maxDisparity;
     // double maxIn, maxDisparity2, maxScaled, maxOut;
     double minVal, maxVal;
     // cv::minMaxLoc(depth, &min, &maxIn);
     depth.convertTo(tmp, CV_64F);
     cv::Mat dMeters = tmp*(this->_dscale);
     // cv::minMaxLoc(dMeters, &min, &maxMeter);
     // cv::Mat dMetersNorm = dMeters*(1.0/->_dscale);
     // cv::minMaxLoc(dMeters, &min, &maxMeter);
     if(debug){
          cv::Mat scaledDisparity, dispRaw, dispMeters;
          double maxIn;
          cv::convertScaleAbs(depth, dispRaw, 255 / maxIn);
          cv::applyColorMap(dispRaw, dispRaw, cv::COLORMAP_JET); cv::imshow("DepthIn", dispRaw);

          dMeters.convertTo(dispMeters, CV_32F);
          cv::applyColorMap(dispMeters, dispMeters, cv::COLORMAP_JET); cv::imshow("MetersIn", dispMeters);
          // printf("[INFO] CameraD415::convert_to_disparity() ---- Depth Limits Before: min = %.3f -- max = %.3f\r\n", min,max);
     }

     cv::Mat zerosMask = cv::Mat(dMeters == 0);
     dMeters.setTo(1, zerosMask);
     disparity = (this->_fxd * this->_baseline) / dMeters;
     disparity.setTo(0, zerosMask);
     cv::minMaxLoc(disparity, &minVal, &maxDisparity);
     float dDisparity = (maxDisparity - minVal);
     // float dTrueDisp = (trueMaxDisparity - trueMinDisparity);
     // float disparityRatio = dDisparity / dTrueDisp;
     // float scale = (255.0*disparityRatio) / dDisparity;
     float scale = (255.0) / dDisparity;
     // disparity.convertTo(disparity8,CV_8UC1);
     // disparity.convertTo(disparity8,CV_8UC1, scale, -minVal*scale);
     disparity.convertTo(disparity8,CV_8UC1, scale);

     // cvinfo(disparity8,"disparity8");
     // double absRatio = ((double) trueMaxDisparity) / maxDisparity;

     if(debug){
          cv::Mat disp1;
          cv::convertScaleAbs(disparity, disp1, 255 / maxDisparity);
          cv::applyColorMap(disp1, disp1, cv::COLORMAP_JET); cv::imshow("Raw Disparity", disp1);

          disparity.convertTo(tmp, CV_32F);
          cv::normalize(tmp,tmp, 1);
          tmp.convertTo(disp1, CV_8U);
          cv::applyColorMap(disp1, disp1, cv::COLORMAP_JET); cv::imshow("Temp Display", disp1);
     }

     // printf("trueMinDisparity = %.2lf, trueMaxDisparity = %.2lf -- maxDisparity = %.2lf -- absRatio = %.2lf\r\n", trueMinDisparity, trueMaxDisparity,maxDisparity, absRatio);
     // cv::Mat disparity2 = disparity*absRatio;
     // cv::minMaxLoc(disparity2, &minVal, &maxDisparity2);
     // if(debug){
     // }

     double gain = 255.0 / maxDisparity;
     double offset = maxDisparity / 65535.0;
     // double gain2 = 255.0 / maxDisparity2;
     // double offset2 = maxDisparity2 / 65535.0;

     // cv::Scalar avg,sdv;
     // cv::meanStdDev(image, avg, sdv);
     // sdv.val[0] = sqrt(image.cols*image.rows*sdv.val[0]*sdv.val[0]);
     // cv::Mat image_32f;
     // image.convertTo(image_32f,CV_32F,1/sdv.val[0],-avg.val[0]/sdv.val[0]);

     // cv::convertScaleAbs(disparity2, scaledDisparity, 255 / maxDisparity2);
     // cv::minMaxLoc(scaledDisparity, &minVal, &maxScaled);
     // double gainScaled = 255.0 / maxScaled;
     // double offsetScaled = maxScaled / 65535.0;

     // if(use_test) cv::convertScaleAbs(disparity, disparity8, 255 / maxDisparity);
     // else scaledDisparity.convertTo(disparity8,CV_8U,gainScaled);
     // disparity2.convertTo(scaledDisparity,CV_8U,gain2);
     // else disparity.convertTo(disparity8,CV_8U,gain);
     // cv::minMaxLoc(disparity8, &minVal, &maxOut);

     // cv::applyColorMap(disparity, disp, cv::COLORMAP_JET); cv::imshow("Disparity", disp);
     // cv::waitKey(0);


     // printf("[INFO] CameraD415::convert_to_disparity() ---- Max Values: Input (%.2f) --> depth (%.3f) --> disparity (%.2f) --> disparity2 (%.2f) --> scaled disparity (%.2f) --> Output (%.2f)\r\n", maxIn, maxMeter, maxDisparity, maxDisparity2, maxScaled, maxOut);
     if(*conversion_gain) *conversion_gain = gain;
     if(*conversion_offset) *conversion_offset = offset;
     return disparity8;
     // return scaledDisparity;
}
int CameraD415::get_pointcloud(){
     // rs2::pointcloud pc;
     // // We want the points object to be persistent so we can display the last cloud when a frame drops
     // rs2::points points;
     // pc.map_to(color);
     // auto depth = frames.get_depth_frame();
     //
     // // Generate the pointcloud and texture mappings
     // points = pc.calculate(depth);
     return 0;
}

/*
 ██████  ███████ ████████ ████████ ███████ ██████  ███████
██       ██         ██       ██    ██      ██   ██ ██
██   ███ █████      ██       ██    █████   ██████  ███████
██    ██ ██         ██       ██    ██      ██   ██      ██
 ██████  ███████    ██       ██    ███████ ██   ██ ███████
*/

vector<rs2::device> CameraD415::get_available_devices(bool show_features, bool verbose){
     int index = 0;
     vector<rs2::device> devs;
     rs2::device_list devices = this->_ctx.query_devices();
     for(rs2::device device : devices){
          std::string name = "Unknown Device";
          std::string sn = "########";
          if(device.supports(RS2_CAMERA_INFO_NAME)) name = device.get_info(RS2_CAMERA_INFO_NAME);
          if(device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) sn = std::string("#") + device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
          if(verbose) std::cout << "  " << index++ << " : " << name + " " + sn << std::endl;
          if(show_features) get_available_sensors(device);
          devs.push_back(device);
     }
     return devs;
}
std::string CameraD415::get_sensor_name(const rs2::sensor& sensor){
     if (sensor.supports(RS2_CAMERA_INFO_NAME)) return sensor.get_info(RS2_CAMERA_INFO_NAME);
     else return "Unknown Sensor";
}
void CameraD415::get_available_sensors(rs2::device dev){
     int index = 0;
     std::vector<rs2::sensor> sensors = dev.query_sensors();

     std::cout << "Device consists of " << sensors.size() << " sensors:\n" << std::endl;
     for(rs2::sensor sensor : sensors){
          std::cout << "  " << index++ << " : " << get_sensor_name(sensor) << std::endl;
     }

     uint32_t selected_sensor_index = get_user_selection("Select a sensor by index: ");
     // The second way is using the subscript ("[]") operator:
     if(selected_sensor_index >= sensors.size()){
          throw std::out_of_range("Selected sensor index is out of range");
     }
     get_sensor_option(sensors[selected_sensor_index]);
}
void CameraD415::get_sensor_option(const rs2::sensor& sensor){
     std::cout << "Sensor supports the following options:\n" << std::endl;
     // The following loop shows how to iterate over all available options
     // Starting from 0 until RS2_OPTION_COUNT (exclusive)
     for(int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++){
          rs2_option option_type = static_cast<rs2_option>(i);
          //SDK enum types can be streamed to get a string that represents them
          std::cout << "  " << i << ": " << option_type;

          // First, verify that the sensor actually supports this option
          if(sensor.supports(option_type)){
               std::cout << std::endl;

               // Get a human readable description of the option
               const char* description = sensor.get_option_description(option_type);
               std::cout << "       Description   : " << description << std::endl;

               // Get the current value of the option
               float current_value = sensor.get_option(option_type);
               std::cout << "       Current Value : " << current_value << std::endl;

          } else std::cout << " is not supported" << std::endl;
     }
     uint32_t selected_sensor_option = get_user_selection("Select an option by index: ");
     if (selected_sensor_option >= static_cast<int>(RS2_OPTION_COUNT)){
          throw std::out_of_range("Selected option is out of range");
     }
}
int CameraD415::get_intrinsics(rs2_stream stream_type, cv::Mat* K, cv::Mat* P, bool verbose){
     int err;
     cv::Mat _K = cv::Mat::zeros(3, 3, CV_64F);
     cv::Mat _P = cv::Mat::zeros(3, 4, CV_64F);

     if(this->_profile){
          rs2::video_stream_profile tmp_stream = this->_profile.get_stream(stream_type).as<rs2::video_stream_profile>();
          rs2_intrinsics intr = tmp_stream.get_intrinsics();

          _K.at<double>(0) = intr.fx;
          _K.at<double>(2) = intr.ppx;
          _K.at<double>(4) = intr.fy;
          _K.at<double>(5) = intr.ppy;
          _K.at<double>(8) = 1.0;

          _P.at<double>(0) = _K.at<double>(0);
          _P.at<double>(2) = _K.at<double>(2);
          _P.at<double>(5) = _K.at<double>(4);
          _P.at<double>(6) = _K.at<double>(5);
          _P.at<double>(10) = 1;

          if(verbose){
               printf("[INFO] CameraD415::get_intrinsics() --- Intrinsic Properties:\r\n");
               printf("\tSize ---------- [w, h]: %d, %d\r\n",tmp_stream.width(),tmp_stream.height());
               printf("\tFocal Length -- [X, Y]: %.2f, %.2f\r\n",intr.fx,intr.fy);
               printf("\tPrinciple Point [X, Y]: %.2f, %.2f\r\n",intr.ppx,intr.ppy);
               std::cout << "K = "<< std::endl << " "  << _K << std::endl << std::endl;
          }
          err = 1;
     } else{ err = -1; }
     *K = _K;
     *P = _P;
     return err;
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
          this->_dscale = scale;
          if(verbose) printf("[INFO] CameraD415::get_depth_scale() --- Depth Scale = %f\r\n",scale);
          return scale;
     }
}
std::vector<rs2::filter> CameraD415::get_default_filters(bool use_decimation, bool use_threshold,
     bool depth_to_disparity, bool use_spatial, bool use_temporal, bool use_hole_filling)
{

     std::vector<rs2::filter> filters;

     /** Decimation - reduces depth frame density */
     if(use_decimation){
          rs2::decimation_filter decimate_filter(1.0f);
          // filters.emplace_back("Decimate", decimate_filter);
          filters.push_back(decimate_filter);
     }
     /** Threshold  - removes values outside recommended range */
     if(use_threshold){
          rs2::threshold_filter thresh_filter(0.1f, D415_MAX_DEPTH_M);
          // filters.emplace_back("Threshold", thresh_filter);
          filters.push_back(thresh_filter);
     }

     if(depth_to_disparity){
          rs2::disparity_transform depth_to_disparity(true);
          // filters.emplace_back(this->disparity_filter_name, depth_to_disparity);
          filters.push_back(depth_to_disparity);
     }
     /** Spatial    - edge-preserving spatial smoothing */
     if(use_spatial){
          rs2::spatial_filter spatial_filter(0.5f, 20.0f, 2.0f, 3.0f);
          // filters.emplace_back("Spatial", spatial_filter);
          filters.push_back(spatial_filter);
     }
     /** Temporal   - reduces temporal noise */
     if(use_temporal){
          rs2::temporal_filter temporal_filter(0.4f, 40.0f, 0);
          // filters.emplace_back("Temporal", temporal_filter);
          filters.push_back(temporal_filter);
     }

     if(use_hole_filling){
          rs2::hole_filling_filter hole_filler(1);
          // filters.emplace_back("Hole-Filling", hole_filler);
          filters.push_back(hole_filler);
     }

     if(depth_to_disparity){
          rs2::disparity_transform disparity_to_depth(false);
          // filters.emplace_back(this->depth_filter_name, disparity_to_depth);
          filters.push_back(disparity_to_depth);
     }
     return filters;
}

/*
███████ ███████ ████████ ████████ ███████ ██████  ███████
██      ██         ██       ██    ██      ██   ██ ██
███████ █████      ██       ██    █████   ██████  ███████
     ██ ██         ██       ██    ██      ██   ██      ██
███████ ███████    ██       ██    ███████ ██   ██ ███████
*/

void CameraD415::enable_filters(){ this->_do_processing = true; }
void CameraD415::disable_filters(){ this->_do_processing = false; }
void CameraD415::enable_alignment(){ this->_do_align = true; }
void CameraD415::disable_alignment(){ this->_do_align = false; }
void CameraD415::enable_timing_debug(){ this->_debug_timings = true; }
void CameraD415::disable_timing_debug(){ this->_debug_timings = false; }

int CameraD415::get_raw_queued_images(cv::Mat* rgb, cv::Mat* depth){
     int err = 0;
     int errFrames = 0;
     rs2::frameset tmpSet;
     cv::Mat _rgb, _depth, _disparity;

     this->_raw_queue.poll_for_frame(&tmpSet);
     if(tmpSet){
          rs2::frame _frgb = tmpSet.get_color_frame();
          rs2::frame _fdepth = tmpSet.get_depth_frame();
          if(_frgb) *rgb = cv::Mat(cv::Size(_cwidth, _cheight), CV_8UC3, (void*)_frgb.get_data(), cv::Mat::AUTO_STEP);
          else *rgb = cv::Mat::zeros(_cwidth, _cheight, CV_8UC3);

          if(_fdepth) *depth = cv::Mat(cv::Size(_dwidth, _dheight), CV_16UC1, (void*)_fdepth.get_data(), cv::Mat::AUTO_STEP);
          else *depth = cv::Mat::zeros(_dwidth, _dheight, CV_16UC1);
          return 0;
     } else return -1;
     return -2;
}

int CameraD415::get_processed_queued_images(cv::Mat* rgb, cv::Mat* depth){
     int err = 0;
     int errFrames = 0;
     rs2::frameset tmpSet;
     cv::Mat _rgb, _depth, _disparity;

     this->_proc_queue.poll_for_frame(&tmpSet);
     if(tmpSet){
          rs2::frame _frgb = tmpSet.get_color_frame();
          rs2::frame _fdepth = tmpSet.get_depth_frame();
          if(_frgb) *rgb = cv::Mat(cv::Size(_cwidth, _cheight), CV_8UC3, (void*)_frgb.get_data(), cv::Mat::AUTO_STEP);
          else *rgb = cv::Mat::zeros(_cwidth, _cheight, CV_8UC3);

          if(_fdepth) *depth = cv::Mat(cv::Size(_dwidth, _dheight), CV_16UC1, (void*)_fdepth.get_data(), cv::Mat::AUTO_STEP);
          else *depth = cv::Mat::zeros(_dwidth, _dheight, CV_16UC1);
          return 0;
     } else return -1;
     return -2;
}

// int CameraD415::get_queued_images(cv::Mat* rgb, cv::Mat* depth, cv::Mat* disparity, bool get_disparity){
//      int err = 0;
//      int errFrames = 0;
//      rs2::frameset tmpSet;
//      cv::Mat _rgb, _depth, _disparity;
//
//      this->_proc_queue.poll_for_frame(&tmpSet);
//      if(tmpSet){
//           rs2::frame _frgb = tmpSet.get_color_frame();
//           rs2::frame _fdepth = tmpSet.get_depth_frame();
//           if(_frgb) *rgb = cv::Mat(cv::Size(_cwidth, _cheight), CV_8UC3, (void*)_frgb.get_data(), cv::Mat::AUTO_STEP);
//           else *rgb = cv::Mat::zeros(_cwidth, _cheight, CV_8UC3);
//
//           if(_fdepth){
//                *depth = cv::Mat(cv::Size(_dwidth, _dheight), CV_16UC1, (void*)_fdepth.get_data(), cv::Mat::AUTO_STEP);
//                if(get_disparity){
//                     rs2::frame _fdisparity = this->_depth2disparity.process(_fdepth);
//                     *disparity = cv::Mat(cv::Size(_dwidth, _dheight), CV_16UC1, (void*)_fdisparity.get_data(), cv::Mat::AUTO_STEP);
//                }
//                else *disparity = cv::Mat::zeros(_dwidth, _dheight, CV_16UC1);
//           }
//           else{
//                *depth = cv::Mat::zeros(_dwidth, _dheight, CV_16UC1);
//                *disparity = cv::Mat::zeros(_dwidth, _dheight, CV_16UC1);
//           }
//           return 0;
//      } else return -1;
//      return -2;
// }

void CameraD415::processingThread(){
     float dt;
     int err = 0;
     int step = 0;
     bool goodRgb = false;
     bool goodDepth = false;
     bool goodProcessed = false;
     rs2::frame color_frame, depth_frame, processed_frame;

     rs2::decimation_filter decimate_filter(1.0f);
     rs2::threshold_filter thresh_filter(0.1f, 10.0f);
     rs2::disparity_transform depth_to_disparity(true);
     rs2::spatial_filter spatial_filter(0.5f, 20.0f, 2.0f, 3.0f);
     rs2::temporal_filter temporal_filter(0.4f, 40.0f, 0);
     rs2::hole_filling_filter hole_filler(1);
     rs2::disparity_transform disparity_to_depth(false);

	high_resolution_clock::time_point _prev_time = high_resolution_clock::now();
	high_resolution_clock::time_point now;
	duration<float> time_span;
     while(!this->_stopped){
          if(this->_debug_timings){
               now = high_resolution_clock::now();
               time_span = duration_cast<duration<float>>(now - _prev_time);
               dt = time_span.count();
               // printf(" --- %.7f (%.2f) ---- \r\n",dt, (1/dt));
               printf("[INFO] CameraD415::processingThread() ---- Starting Step %d (previous step took %.2f sec [%.2f Hz]):\r\n", step,dt, (1/dt));
          }
          // else printf("[INFO] CameraD415::processingThread() ---- Starting Step %d:\r\n", step);

          rs2::frameset data;
          if(this->_pipe.poll_for_frames(&data)){
               if(this->_do_align) data = data.apply_filter(*this->_align);
               rs2::frameset raw = data;

               rs2::frameset processed;
               if(this->_do_processing) this->process_frames(data,&processed);

               this->_raw_queue.enqueue(raw);
               this->_proc_queue.enqueue(processed);
               if(this->_debug_timings){ _prev_time = now; }
               step++;
          }
     }
     printf("[INFO] CameraD415::processingThread() ---- Exiting loop...\r\n");
     this->_thread_started = false;
}
