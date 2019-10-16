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
     int _fps = 30;

     float _fx;
     float _fy;
     float _ppx;
     float _ppy;
     float _baseline;
     float _depth_scale;

     uint64_t _counter = 0;
     uint64_t _img_counter = 0;
public:
     cv::Mat intrinsic_calib;
     cv::Mat distortion_calib;
     rs2::colorizer color_map;

	CameraD415();
	CameraD415(int height, int width, int fps);
     ~CameraD415();

     bool start(int height, int width, int fps);
     bool stop();
     bool reset(int height, int width, int fps, bool with_startup = true);

     cv::Mat get_intrinsics(bool verbose = false);
     void get_extrinsics(bool verbose = false);
     float get_baseline(bool verbose = false);
     float get_depth_scale(bool verbose = false);

     cv::Mat get_rgb_image();
     cv::Mat get_depth_image();
     cv::Mat convert_to_disparity(const cv::Mat depth, double& conversion_gain);
     vector<cv::Mat> read();
     int read(cv::Mat& rgb, cv::Mat& depth);

     void update();
     vector<rs2::device> get_available_devices(bool verbose = false);
};

#endif /* CAMERA_D415_H_*/
