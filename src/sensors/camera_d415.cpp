#include <iostream>
#include <unistd.h>                // For usleep
#include <limits>                  // For infinity
#include <math.h>                  // For fabs

#include "sensors/camera_d415.h"
#include "utilities/cv_utils.h"
#include "utilities/image_utils.h"

using namespace std;
using namespace chrono;

template<typename Pixel>
struct ForEachOperator{
     Pixel m_gain;
     ForEachOperator(Pixel gain){
          m_gain = gain;
     }
     void operator()(Pixel& pixel, const int * idx) const {
          if(pixel != 0.0){
               pixel = m_gain / pixel;
          }
     }
};

filter_options::filter_options(const std::string name, rs2::filter& flt) :
    filter_name(name), filter(flt), is_enabled(true){}

filter_options::filter_options(filter_options&& other) :
    filter_name(std::move(other.filter_name)), filter(other.filter),
    is_enabled(other.is_enabled.load()){}

CameraD415::CameraD415(bool use_callback, bool show_options) : _cam_thread(), _stopped(false),
     _do_align(false), _do_processing(false), _thread_started(false), _debug_timings(false), _depth2disparity(true), _disparity2depth(false),
     _proc_queue(MAX_QUEUE), _raw_queue(MAX_QUEUE), _disparity_queue(MAX_QUEUE), _align(RS2_STREAM_DEPTH), _use_callback(use_callback)
{
     int err;
     bool verbose = true;

     vector<rs2::device> devList = this->get_available_devices(show_options, true);
     this->_dev = devList[0];
     this->_device_name = this->_dev.get_info(RS2_CAMERA_INFO_NAME);

     RS_STREAM_CFG rgb_cfg = {RS2_STREAM_COLOR, _cwidth, _cheight, RS2_FORMAT_BGR8, _cfps};
     RS_STREAM_CFG depth_cfg = {RS2_STREAM_DEPTH, _dwidth, _dheight, RS2_FORMAT_Z16, _dfps};
     std::vector<RS_STREAM_CFG> cfgs = {rgb_cfg, depth_cfg};

     if(this->start_streams(cfgs, use_callback)) printf("SUCCESS: D415 camera streams initialized!\r\n");
     else exit(0);

     // this->_align = new rs2::align(RS2_STREAM_COLOR);
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

     this->_dscale = this->_get_depth_scale(verbose);
     this->_baseline = this->_get_baseline(verbose);
     this->_trueMinDisparity = (this->_fxd * this->_baseline)/((float) D415_MIN_DEPTH_M);
     this->_trueMaxDisparity = (this->_fxd * this->_baseline)/((float) D415_MAX_DEPTH_M);
     this->_filters = this->get_default_filters();
     printf("[INFO] CameraD415::CameraD415() --- Successfully created.\r\n");
}

CameraD415::CameraD415(int rgb_fps, int rgb_resolution[2], int depth_fps, int depth_resolution[2],
     bool use_callback, bool show_options) : _cam_thread(), _stopped(false), _do_align(false), _debug_timings(false),
     _do_processing(false), _thread_started(false), _depth2disparity(true), _disparity2depth(false),
     _proc_queue(MAX_QUEUE), _raw_queue(MAX_QUEUE), _disparity_queue(MAX_QUEUE), _align(RS2_STREAM_DEPTH), _use_callback(use_callback)
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

     if(this->start_streams(cfgs, use_callback)) printf("SUCCESS: D415 camera streams initialized!\r\n");
     else exit(0);

     // this->_align = new rs2::align(RS2_STREAM_COLOR);
     // this->_align = new rs2::align(RS2_STREAM_DEPTH);
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

     this->_dscale = this->_get_depth_scale(verbose);
     this->_baseline = this->_get_baseline(verbose);
     this->_trueMinDisparity = (this->_fxd * this->_baseline)/((float) D415_MIN_DEPTH_M);
     this->_trueMaxDisparity = (this->_fxd * this->_baseline)/((float) D415_MAX_DEPTH_M);
     this->_filters = this->get_default_filters();
     printf("[INFO] CameraD415::CameraD415() --- Successfully created.\r\n");
}

CameraD415::~CameraD415(){
     this->stop();
     // delete this->_align;
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
bool CameraD415::start_streams(std::vector<RS_STREAM_CFG> stream_cfgs, bool use_callback){
     bool err = this->reset(stream_cfgs, false, use_callback);
     usleep(STARTUP_DELAY_SEC * 1000000);
     // Try initializing camera hardware
     if(!this->hardware_startup(stream_cfgs, use_callback)){
          // Attempt to initialize hardware with a reset, if unsuccessful initialization
          if(!this->reset(stream_cfgs, true, use_callback)){
               // if unsuccessful initialization after two reset attempts give up
               if(!this->reset(stream_cfgs, true, use_callback)){
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
bool CameraD415::hardware_startup(std::vector<RS_STREAM_CFG> stream_cfgs, bool use_callback){
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
          rs2::pipeline_profile profile;
          if(use_callback){
               auto callback = [&](const rs2::frame& frame){
                    std::lock_guard<std::mutex> lock(this->_lock);
                    rs2::frameset processed;
                    double t = (double)cv::getTickCount();
                    if( rs2::frameset data = frame.as<rs2::frameset>() ){
                       if(this->_do_align) data = this->_align.process(data);
                       this->process_frames(data,&processed, this->_do_processing);
                       this->_proc_queue.enqueue(processed);
                       if(this->_debug_timings){
                            double dt = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
                            t = (double)cv::getTickCount();
                            printf("[INFO] CameraD415::processingCallback() ---- Starting Step %d (previous step took %.2lf sec [%.2lf Hz]):\r\n", this->_callback_counter,dt, (1/dt));
                       }
                       this->_callback_counter++;
                   }
              };
               profile = this->_pipe.start(this->_cfg, callback);
          } else profile = this->_pipe.start(this->_cfg);
          this->_profile = profile;
     } catch(const rs2::error & e){
          std::cerr << "[ERROR] CameraD415::hardware_startup() --- Could not initialize rs2::pipeline_profile. RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
          return false;
     }
     return true;
}
bool CameraD415::reset(std::vector<RS_STREAM_CFG> stream_cfgs, bool with_startup, bool use_callback){
     if(this->_dev) this->_dev.hardware_reset();
     else{
          printf("[ERROR] CameraD415::reset() --- No device to reset.\r\n");
          return false;
     }
     if(with_startup) return this->start_streams(stream_cfgs, use_callback);
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
     if(flag_aligned) this->_aligned_frames = this->_align.process(this->_frames);
     // if(flag_aligned) this->_aligned_frames = this->_align->process(this->_frames);

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
int CameraD415::process_frames(rs2::frameset frames, rs2::frameset* processed, bool use_filters){
     int err = 0;
     rs2::frameset _processed = frames;
     if((this->_filters.size() > 0) && (use_filters)){
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
/** BETA TESTING FUNCTION */
cv::Mat CameraD415::convert_to_disparity(const cv::Mat depth, double* conversion_gain, double* conversion_offset){
     cv::Mat tmp, disparity8;
     double min, maxIn, maxDisparity, maxDepth;
     // cvinfo(depth,"depth input");
     depth.convertTo(tmp, CV_64F);
     tmp = tmp * this->_dscale;
     cv::minMaxLoc(tmp, &min, &maxDepth);
     // cvinfo(tmp,"depth input to CV_64F");

     cv::Mat disparity = cv::Mat((this->_fxd * this->_baseline) / tmp);
     // cvinfo(disparity,"disparity");
     cv::minMaxLoc(disparity, &min, &maxDisparity);
     double gain = 255.0 / maxDepth;
     double ratio = this->_trueMinDisparity / maxDisparity;
     double ratiodepth = this->_trueMinDisparity / maxDepth;
     double offset = ratio / gain;
     double tmpgain = 255.0 / (double)(this->_trueMinDisparity);
     int tmpVal = (tmpgain*maxDisparity);
     double delta = 255.0 / (double)(tmpVal);
     // float tmpgain = (float)(ratio*256.0) / this->_trueMinDisparity;
     // float tmpgain = 256.0 / maxDisparity;
     disparity.convertTo(disparity8,CV_8UC1, tmpgain);
     if(false){
          cvinfo(disparity8,"disparity8 pregain");
          printf(" --- tmpgain=%.2f | ratio=%.2f | maxDisparity=%.2f | delta=%.2f | tmpVal=%d --- \r\n",tmpgain, ratio, maxDisparity, delta,tmpVal);
          // disparity8.convertTo(disparity8,CV_8UC1, ratio);
          cv::Mat disparity88 = cv::Mat_<uchar>(disparity8 * delta);
          cvinfo(disparity88,"disparity8 postgain");
     }
     // disparity.convertTo(disparity8,CV_8UC1, 255.0/this->_trueMinDisparity);
     // cvinfo(tmpDisp,"tmpDisp");
     // cvinfo(disparity8,"disparity8");

     // cv::minMaxLoc(disparity8, &min, &maxIn);
     // double dmaxHat = maxIn * offset;
     // printf(" --- maxDisparity=%.2f | maxDepth=%.2f | trueMaxDisparity=%.2f | derivedMaxDepth=%.2f --- \r\n",maxDisparity, maxDepth, this->_trueMinDisparity, dmaxHat);

     // cv::imshow("disparity", disparity);
     // cv::imshow("tmpDisp", tmpDisp);
     // cv::imshow("disparity8", disparity8);

     if(*conversion_gain) *conversion_gain = tmpgain*ratio;
     if(*conversion_offset) *conversion_offset = maxDepth;
     return disparity8;
     // return disparity88;
}

cv::Mat CameraD415::convert_to_disparity_alternative(const cv::Mat depth, double* conversion_gain, double* conversion_offset){
     cv::Mat tmp, disparity8;
     int roiSz = 5;
     cv::Rect microRoi = cv::Rect(0,depth.rows-2*roiSz,roiSz,roiSz);
     double min, maxIn, maxDisparity, minDisparity, maxDepth;
     // cvinfo(depth,"depth input");
     depth.convertTo(tmp, CV_64F);
     tmp = tmp * this->_dscale;
     cv::minMaxLoc(tmp, &min, &maxDepth);
     // cvinfo(tmp,"depth input to CV_64F");
     // tmp(microRoi).setTo(65.535);
     tmp(microRoi).setTo(double(this->_fxd * this->_baseline)/65.535);
     cv::Rect microRoi2 = cv::Rect(0,depth.rows-roiSz,roiSz,roiSz);
     tmp(microRoi2).setTo(double(this->_fxd * this->_baseline)/0.001);
     cv::Mat disparity = cv::Mat(1.0 / tmp);
     disparity = disparity * double(this->_fxd * this->_baseline);
     cvinfo(disparity,"disparity");
     cv::minMaxLoc(disparity, &minDisparity, &maxDisparity);
     double gain = 256.0 / maxDepth;
     double ratio = (double)(this->_trueMinDisparity) / maxDisparity;
     double offset = ratio / gain;
     // double tmpgain = 256.0 / (double)(this->_trueMinDisparity);
     double tmpgain = 256.0 / maxDisparity;
     // double tmpgain = (double)(256.0*this->_trueMinDisparity) / ();
     int tmpVal = (tmpgain*maxDisparity);
     double delta = 256.0 / (double)(tmpVal);
     // float tmpgain = (float)(ratio*256.0) / this->_trueMinDisparity;
     // float tmpgain = 256.0 / maxDisparity;
     // cv::Mat disparityTest = cv::Mat(disparity*ratio);
     // // cv::Mat disparityTest = cv::Mat(disparity/maxDisparity);
     // cvinfo(disparityTest,"\tdisparityTest");
     disparity.convertTo(disparity8,CV_8UC1, tmpgain);
     // cv::convertScaleAbs(disparity, disparity8, 255.0/maxDisparity);
     cvinfo(disparity8,"\tdisparityTest8 pregain");
     printf(" --- tmpgain=%.2f | ratio=%.2f | maxDisparity=%.2f | minDisparity=%.2f | tmpVal=%d --- \r\n",tmpgain, ratio, maxDisparity, minDisparity,tmpVal);
     // disparity8.convertTo(disparity8,CV_8UC1, ratio);
     // cvinfo(disparity8,"disparity8 postgain");
     // disparity.convertTo(disparity8,CV_8UC1, 255.0/this->_trueMinDisparity);
     // cvinfo(tmpDisp,"tmpDisp");
     // cvinfo(disparity8,"disparity8");

     // cv::minMaxLoc(disparity8, &min, &maxIn);
     // double dmaxHat = maxIn * offset;
     // printf(" --- maxDisparity=%.2f | maxDepth=%.2f | trueMaxDisparity=%.2f | derivedMaxDepth=%.2f --- \r\n",maxDisparity, maxDepth, this->_trueMinDisparity, dmaxHat);

     // cv::imshow("disparity", disparity);
     // cv::imshow("tmpDisp", tmpDisp);
     // cv::imshow("disparity8", disparity8);

     if(*conversion_gain) *conversion_gain = offset;
     if(*conversion_offset) *conversion_offset = delta;
     return disparity8;
}

/** ALPHA TESTING FUNCTION */
/**
cv::Mat CameraD415::convert_to_disparity(const cv::Mat depth, double* conversion_gain, double* conversion_offset){
     // float trueMaxDisparity = (this->_fxd * this->_baseline)/((float) D415_MAX_DEPTH_M);
     // float trueMinDisparity = (this->_fxd * this->_baseline)/((float) D415_MIN_DEPTH_M);
     // float cvter = (this->_fxd * this->_baseline) / (this->_dscale);
     // float cvter = (this->_fxd * this->_baseline);

     // cv::Mat zerosMask = cv::Mat(depth == 0);
     cv::Mat tmp, disparity8;
     double min, maxIn, maxDisparity;
     // double maxIn, maxDisparity2, maxScaled, maxOut;
     // cvinfo(depth,"depth input");
     depth.convertTo(tmp, CV_64F);
     tmp = tmp * this->_dscale;
     // cvinfo(tmp,"depth input to CV_64F");
     cv::minMaxLoc(tmp, &min, &maxIn);

     // cv::Mat dMeters;
     // cv::Mat tmp2 = tmp / maxIn;
     // cvinfo(tmp2,"tmp / max(tmp)");
     // cv::Mat dMeters = cv::Mat(1.0 / tmp);
     // // cv::Mat dMeters = cv::Mat(1.0 / tmp2);
     // // cv::minMaxLoc(dMeters, &minMeter, &maxMeter);
     // // cvinfo(dMeters,"dMeters");

     cv::Mat disparity = cv::Mat((this->_fxd * this->_baseline) / tmp);
     cv::minMaxLoc(disparity, &min, &maxDisparity);
     float scale = (255.0) / maxDisparity;
     // cv::Mat tmpDisp = disparity * scale;
     // tmpDisp.convertTo(disparity8,CV_8UC1);
     disparity.convertTo(disparity8,CV_8UC1,scale);
     // cvinfo(tmpDisp,"tmpDisp");
     // cvinfo(disparity8,"disparity8");
     // printf(" --- dDisparity=%.2f  --- \r\n", dDisparity);
     // cv::imshow("disparity", disparity);
     // cv::imshow("tmpDisp", tmpDisp);
     // cv::imshow("disparity8", disparity8);

     double gain = 255.0 / maxDisparity;
     double offset = 255.0 / maxIn;

     if(*conversion_gain) *conversion_gain = gain;
     if(*conversion_offset) *conversion_offset = offset;
     return disparity8;
}*/
/** BASE FUNCTION */
/** */
cv::Mat CameraD415::convert_to_disparity_test(const cv::Mat depth, double* conversion_gain, double* conversion_offset){
     cv::Mat tmpMat;
     depth.convertTo(tmpMat, CV_32F);
     double gain = (this->_fxd * this->_baseline) / this->_dscale;
     double maxDisparity;
     /** Initial Attempt Method */
     {
          // cv::Mat dMeters;
          // cv::Mat zerosMask = cv::Mat(depth == 0.0);
          //
          // depth.convertTo(dMeters, CV_64F,this->_dscale);
          // dMeters.setTo(1.0, zerosMask);
          //
          // cv::Mat disparity = cv::Mat((double)(this->_fxd * this->_baseline) / dMeters);
          //
          // double minDisparity, maxDisparity;
          // cv::minMaxLoc(disparity, &minDisparity, &maxDisparity);
          // disparity.setTo(0.0, zerosMask);
          //
          // double gain = 255.0 / (maxDisparity);
          // disparity.convertTo(tmpMat,CV_8UC1, gain);
     }

     /** Naive pointer access method */
     {
          // float* pix;
          // for(int v = 0; v < depth.rows; ++v){
          //      pix = tmpMat.ptr<float>(v);
          //      for(int u = 0; u < depth.cols; ++u){
          //           float dvalue = pix[u];
          //           if(dvalue != 0.0) pix[u] = ((float) gain / dvalue);
          //      }
          // }
     }

     ForEachOperator<float> initializer((float)gain);
     tmpMat.forEach<float>(initializer);

     if(conversion_gain) *conversion_gain = gain;
     if(conversion_offset) *conversion_offset = maxDisparity;
     return tmpMat;
}
int CameraD415::get_pointcloud(const rs2::frame& depth, rs2::points* cloud){
     // Generate the pointcloud and texture mappings
     rs2::points points = this->pc_.calculate(depth);
     if(*cloud) *cloud = points;
     return 0;
}
int CameraD415::get_pointcloud(const rs2::frame& depth, const rs2::frame& color, rs2::points* cloud){
     this->pc_.map_to(color);
     // Generate the pointcloud and texture mappings
     rs2::points points = this->pc_.calculate(depth);
     if(*cloud) *cloud = points;
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
float CameraD415::_get_baseline(bool verbose){
     if(this->_profile){
          // rs2::stream_profile ir1_stream = this->_profile.get_stream(RS2_STREAM_INFRARED, 1);
          // rs2::stream_profile ir2_stream = this->_profile.get_stream(RS2_STREAM_INFRARED, 2);
          // rs2_extrinsics extr = ir1_stream.get_extrinsics_to(ir2_stream);
          rs2::stream_profile depth_stream = this->_profile.get_stream(RS2_STREAM_DEPTH);
          rs2::stream_profile rgb_stream = this->_profile.get_stream(RS2_STREAM_COLOR);
          rs2_extrinsics extr = depth_stream.get_extrinsics_to(rgb_stream);

          float baseline = extr.translation[0];
          if(verbose) printf("[INFO] CameraD415::_get_baseline() --- Baseline = %f\r\n",baseline);
          return baseline;
     }else{ return -1.0;}
}
float CameraD415::get_baseline(bool verbose){ return this->_baseline; }
float CameraD415::_get_depth_scale(bool verbose){
     if(!this->_profile){
          printf("[ERROR] CameraD415::get_depth_scale() --- Camera Profile is not initialized.\r\n");
          return -1.0;
     } else{
          rs2::depth_sensor sensor = this->_profile.get_device().first<rs2::depth_sensor>();
          float scale = sensor.get_depth_scale();
          if(verbose) printf("[INFO] CameraD415::get_depth_scale() --- Depth Scale = %f\r\n",scale);
          return scale;
     }
}
float CameraD415::get_depth_scale(bool verbose){ return this->_dscale; }

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

void CameraD415::processingThread(){
     int step = 0;
     rs2::frameset data;
     rs2::frameset processed;
     double dt;
     double t = (double)cv::getTickCount();
     while(!this->_stopped){
          t = (double)cv::getTickCount();
          // data = this->_pipe.wait_for_frames();
          // if(data){
          if(this->_pipe.poll_for_frames(&data)){
               this->_lock.lock();
               if(this->_do_align) data = this->_align.process(data);
               // if(this->_do_align) data = data.apply_filter(*this->_align);

               // rs2::frameset raw = data;
               // this->_raw_queue.enqueue(raw);

               // rs2::frameset processed;
               // if(this->_do_processing) this->process_frames(data,&processed);
               // else processed = data;
               this->process_frames(data,&processed, this->_do_processing);
               this->_proc_queue.enqueue(processed);
               if(this->_debug_timings){
                    dt = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
                    printf("[INFO] CameraD415::processingThread() ---- Starting Step %d (previous step took %.2lf sec [%.2lf Hz]):\r\n", step,dt, (1/dt));
               }
               step++;
               this->_lock.unlock();
          } else{
               // usleep(1.0*1000);
               // printf("[INFO] CameraD415::processingThread() ---- Sleeping loop...\r\n");
               std::this_thread::yield();
               std::this_thread::sleep_for(std::chrono::milliseconds(10));
               // if(this->_debug_timings){
               //      dt = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
               //      printf("[INFO] CameraD415::processingThread() ---- End Step %d (previous step took %.2lf sec [%.2lf Hz]):\r\n", step,dt, (1/dt));
               // }
          }
     }
     printf("[INFO] CameraD415::processingThread() ---- Exiting loop...\r\n");
     this->_thread_started = false;
}
