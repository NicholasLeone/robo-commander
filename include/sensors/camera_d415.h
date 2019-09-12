#ifndef CAMERA_D415_H_
#define CAMERA_D415_H_

#include <string>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

// find_package(realsense2 2.25.0)
// if(NOT realsense2_FOUND)
//     message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
// endif()
// ${realsense2_INCLUDE_DIR}
// ${realsense2_LIBRARY}

using namespace std;

class CameraD415{
private:
     string _dev;
public:
     cv::Mat intrinsic_calib;
     cv::Mat distortion_calib;
     cv::Mat prev_img;
     cv::Mat cur_img;
     float focal;
     cv::Point2d pp;

	CameraD415();
	CameraD415(const string& dev);
     ~CameraD415();

     int init(const string& dev);
     bool start();
     bool stop();
     bool reset();

     void get_intrinsics();
     void get_extrinsics();
     float get_depth_scale();

     cv::Mat get_rgb_image();
     cv::Mat get_depth_image();
     void grab_images();

     void update();
};

#endif /* CAMERA_D415_H_*/
