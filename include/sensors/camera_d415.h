#ifndef CAMERA_D415_H_
#define CAMERA_D415_H_

#include <string>
#include <chrono>
#include <vector>
#include <thread>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

using namespace std;

class CameraD415{
private:
     rs2::device _dev;
     rs2::context _ctx;
     rs2::pipeline _pipe;
     rs2::config _cfg;
     rs2::pipeline_profile _profile;

     rs2::frame _df;
     rs2::frame _rgbf;

     std::mutex _lock;

     int _width = 640;
     int _height = 480;

     uint64_t _counter = 0;
     uint64_t _img_counter = 0;
public:
     cv::Mat intrinsic_calib;
     cv::Mat distortion_calib;
     rs2::colorizer color_map;
     float focal;

	CameraD415();
     ~CameraD415();

     bool start();
     bool stop();
     bool reset(bool with_startup = true);

     void get_intrinsics(bool verbose = false);
     void get_extrinsics(bool verbose = false);
     float get_baseline(bool verbose = false);
     float get_depth_scale(bool verbose = false);

     cv::Mat get_rgb_image();
     cv::Mat get_depth_image();
     vector<cv::Mat> update_frames();

     void update();
     vector<rs2::device> get_available_devices(bool verbose = false);
};

#endif /* CAMERA_D415_H_*/
