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

     std::mutex _lock;
public:
     cv::Mat intrinsic_calib;
     cv::Mat distortion_calib;
     cv::Mat prev_img;
     cv::Mat cur_img;
     float focal;

	CameraD415();
     ~CameraD415();

     int init(const std::string& dev);
     bool start();
     bool stop();
     bool reset();

     void get_intrinsics();
     void get_extrinsics();
     float get_depth_scale(bool verbose = false);

     cv::Mat get_rgb_image();
     cv::Mat get_depth_image();
     void grab_images();

     void update();
     vector<rs2::device> get_available_devices(bool verbose = false);
};

#endif /* CAMERA_D415_H_*/
