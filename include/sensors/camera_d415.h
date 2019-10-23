#ifndef CAMERA_D415_H_
#define CAMERA_D415_H_

#include <string>
#include <chrono>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>

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

class filter_options
{
public:
    filter_options(const std::string name, rs2::process_interface& filter);
    filter_options(filter_options&& other);
    std::string filter_name;                                   //Friendly name of the filter
    rs2::process_interface& filter;                            //The filter in use
    std::map<rs2_option, filter_slider_ui> supported_options;  //maps from an option supported by the filter, to the corresponding slider
    std::atomic_bool is_enabled;                               //A boolean controlled by the user that determines whether to apply the filter or not
};

class CameraD415{
private:
     // Threading Mechanisms
     std::mutex _lock;
     std::thread cam_thread;
     // Realsense Objects
     rs2::device _dev;
     rs2::context _ctx;
     rs2::pipeline _pipe;
     rs2::config _cfg;
     rs2::pipeline_profile _profile;
     rs2::align* _align;
     rs2::frameset _frames;
     rs2::frameset _aligned_frames;
     rs2::frame _depth_frame;
     rs2::frame _color_frame;
     rs2::frame_queue _rgb_data;
     rs2::frame_queue _raw_data;
     rs2::frame_queue _filtered_data;
     std::vector<filter_options> _filters;

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
     const std::string disparity_filter_name = "Disparity";
     const std::string depth_filter_name = "Disparity2Depth";

     // Flags
     bool _use_filters = false;
     bool _stopped = false;
     // std::atomic_bool _stopped;

     // Counters
     uint64_t _counter = 0;
     uint64_t _img_counter = 0;
     uint64_t _nRgbFrames = 0;
     uint64_t _nDepthFrames = 0;
     uint64_t _nProcFrames = 0;
public:
     /** Constructors */
	CameraD415(bool use_filters = false, bool show_features = false);
	CameraD415(int rgb_fps, int rgb_resolution[2], int depth_fps, int depth_resolution[2], bool use_filters = false, bool show_features = false);
     ~CameraD415();

     /** Startup - Shutdown - Initialization Functions */
     bool stop();
     void start_thread();
     bool start_streams(std::vector<RS_STREAM_CFG> stream_cfgs);
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

     rs2::frame get_depth_frame(bool flag_aligned = false, bool flag_processed = false);
     // int process_depth_frame(const rs2::frame& frame, rs2::frame* processed, bool visualize = false);
     // int process_depth_frame(rs2::frame frame, rs2::frame* processed, bool visualize = false);

     int get_depth_image(rs2::frame frame, cv::Mat* image, bool flag_processed = false);
     int get_depth_image(cv::Mat* image, bool flag_aligned = false, bool flag_processed = false);

     vector<cv::Mat> read(bool flag_aligned = false, bool flag_processed = false);
     int read(cv::Mat* rgb, cv::Mat* depth, cv::Mat* processed, bool flag_aligned = false, bool flag_processed = false);
     int read(rs2::frame_queue& queue, cv::Mat* image, bool is_depth, bool flag_aligned = false, bool flag_processed = false);

     /** Additional Camera Functionality */
     cv::Mat convert_to_disparity(const cv::Mat depth, double* conversion_gain);
     /**TODO*/int get_pointcloud();

     /** Device Getters */
     vector<rs2::device> get_available_devices(bool show_features = false, bool verbose = false);
     void get_available_sensors(rs2::device dev);
     void get_sensor_option(const rs2::sensor& sensor);
     std::string get_sensor_name(const rs2::sensor& sensor);

     // int process_depth_frame(rs2::frame frame, rs2::frame* processed, bool use_decimation = true,
     //      bool use_threshold = false, bool depth_to_disparity = true,
     //      bool use_spatial = true, bool use_temporal = true, bool use_hole_filling = false
     // );
     std::vector<filter_options> get_default_filters(bool use_decimation = true,
          bool use_threshold = false, bool depth_to_disparity = true,
          bool use_spatial = true, bool use_temporal = true, bool use_hole_filling = false
     );

     /** Device Setters */
     void enable_filters();
     void disable_filters();

     /** Misc Functions */
     // void update_depth_queue(const rs2::frame& frame, bool use_filters = false);
     void update_depth_queue(rs2::frame frame, bool use_filters = false);
     int read_from_queue(rs2::frame* raw_frame, rs2::frame* filtered_frame, bool grab_filtered = false);
     void processingThread();
     // rs2::colorizer color_map;
};

#endif /* CAMERA_D415_H_*/
