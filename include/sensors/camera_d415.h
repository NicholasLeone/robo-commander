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

inline uint32_t get_user_selection(const std::string& prompt_message){
     std::cout << "\n" << prompt_message;
     uint32_t input;
     std::cin >> input;
     std::cout << std::endl;
     return input;
}

typedef struct RS_STREAM_CFG{
     rs2_stream stream_type;
     int width;
     int height;
     rs2_format format;
     int fps;
} RS_STREAM_CFG;

class CameraD415{
private:
     // Low-level
     std::mutex _lock;
     // Realsense Objects
     rs2::device _dev;
     rs2::context _ctx;
     rs2::pipeline _pipe;
     rs2::config _cfg;
     rs2::pipeline_profile _profile;
     rs2::align* _align;  // RS2_STREAM_COLOR
     rs2::frameset _frames;
     rs2::frameset _aligned_frames;
     rs2::frame _depth_frame;
     rs2::frame _color_frame;

     // Depth Stream Settings
     int _dwidth = 640;            // width
     int _dheight = 480;           // height
     int _dfps = 60;               // framerate
     float _fxd, _fyd;             // focal lengths
     float _ppxd, _ppyd;           // principle points
     cv::Mat _Kdepth, _Pdepth;     // camera info matrices

     // RGB Stream Settings
     int _cwidth = 640;            // width
     int _cheight = 480;           // height
     int _cfps = 60;               // framerate
     float _fxc, _fyc;             // focal lengths
     float _ppxc, _ppyc;           // principle points
     cv::Mat _Krgb, _Prgb;        // camera info matrices

     // Misc Cam Params
     cv::Mat _Dmat;
     float _dscale;
     float _baseline;
     std::string _device_name;
     std::string _distortion_model = "plumb_bob";

     // Transformation frame ids
     std::string _camera_base_frame = "camera_link";
     std::string _depth_base_frame = "camera_depth_frame";
     std::string _color_base_frame = "camera_color_frame";
     std::string _depth_optical_frame_id = "camera_depth_optical_frame";
     std::string _color_optical_frame_id = "camera_color_optical_frame";
     std::string _aligned_base_frame = "camera_aligned_depth_to_color_frame";

     // Counters
     uint64_t _counter = 0;
     uint64_t _img_counter = 0;
     uint64_t _nRgbFrames = 0;
     uint64_t _nDepthFrames = 0;
public:
     /** Constructors */
	CameraD415(bool show_features = false);
	CameraD415(int rgb_fps, int rgb_resolution[2], int depth_fps, int depth_resolution[2], bool show_features = false);
     ~CameraD415();

     /** Startup - Shutdown - Initialization Functions */
     bool stop();
     bool start(std::vector<RS_STREAM_CFG> stream_cfgs);
     bool sensors_startup(std::vector<RS_STREAM_CFG> stream_cfgs);
     bool hardware_startup(std::vector<RS_STREAM_CFG> stream_cfgs);
     bool reset(std::vector<RS_STREAM_CFG> stream_cfgs, bool with_startup = true);

     /** Camera Info Getters */
     int get_intrinsics(rs2_stream stream_type, cv::Mat* K, cv::Mat* P, bool verbose = false);
     void get_extrinsics(bool verbose = false);
     float get_baseline(bool verbose = false);
     float get_depth_scale(bool verbose = false);

     /** Base Image Functions */
     rs2::frame get_rgb_frame(bool flag_aligned = false);
     int get_rgb_image(rs2::frame frame, cv::Mat* image);
     int get_rgb_image(cv::Mat* image, bool flag_aligned = false);

     rs2::frame get_depth_frame(bool flag_aligned = false);
     int get_depth_image(rs2::frame frame, cv::Mat* image);
     int get_depth_image(cv::Mat* image, bool flag_aligned = false);

     vector<cv::Mat> read(bool flag_aligned = false);
     int read(cv::Mat* rgb, cv::Mat* depth, bool flag_aligned = false);

     /** Additional Camera Functionality */
     cv::Mat convert_to_disparity(const cv::Mat depth, double* conversion_gain);
     /**TODO*/int get_pointcloud();

     /** Misc Functions */
     void update();
     vector<rs2::device> get_available_devices(bool show_features = false, bool verbose = false);
     void get_available_sensors(rs2::device dev);
     void get_sensor_option(const rs2::sensor& sensor);
     std::string get_sensor_name(const rs2::sensor& sensor);

     // rs2::colorizer color_map;
};

#endif /* CAMERA_D415_H_*/
