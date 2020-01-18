#ifndef CAMERA_D415_H_
#define CAMERA_D415_H_

#include <string>
#include <chrono>
#include <vector>
#include <thread>
#include <map>
#include <mutex>
#include <atomic>

#include <opencv2/opencv.hpp>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#define STARTUP_DELAY_SEC 3.0
#define D415_MAX_DEPTH_M 10.0
#define D415_MIN_DEPTH_M 0.1
#define MAX_QUEUE 4

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

class filter_options{
public:
    filter_options(const std::string name, rs2::filter& filter);
    filter_options(filter_options&& other);
    std::string filter_name;                                   //Friendly name of the filter
    rs2::filter& filter;                            //The filter in use
    std::atomic_bool is_enabled;                               //A boolean controlled by the user that determines whether to apply the filter or not
};

class CameraD415{
private:
     /** Threading Mechanisms */
     std::mutex _lock;
     std::thread _cam_thread;
     /** Flags */
     std::atomic_bool _stopped;
     std::atomic_bool _do_align;
     std::atomic_bool _do_processing;
     std::atomic_bool _thread_started;
     std::atomic_bool _debug_timings;
     /** Counters */
     uint64_t _counter = 0;
     uint64_t _img_counter = 0;
     uint64_t _nRgbFrames = 0;
     uint64_t _nDepthFrames = 0;
     uint64_t _nProcFrames = 0;

     /** Realsense Objects */
     rs2::device _dev;
     rs2::context _ctx;
     rs2::pipeline _pipe;
     rs2::config _cfg;
     rs2::align _align;
     rs2::pipeline_profile _profile;
     // rs2::align* _align;
     rs2::frameset _frames;
     rs2::frameset _aligned_frames;
     rs2::frame _depth_frame;
     rs2::frame _color_frame;
     // rs2::frame_queue _rgb_queue;
     // rs2::frame_queue _depth_queue;
     rs2::frame_queue _raw_queue;
     rs2::frame_queue _proc_queue;
     rs2::frame_queue _disparity_queue;
     std::vector<rs2::filter> _filters;
     // std::vector<filter_options> _filters;
     rs2::colorizer _cmap;
     rs2::disparity_transform _depth2disparity;
     rs2::disparity_transform _disparity2depth;
     rs2::pointcloud pc_;

     /** Depth Stream Settings */
     int _dwidth = 848;            // width
     int _dheight = 480;           // height
     int _dfps = 60;               // framerate
     float _fxd, _fyd;             // focal lengths
     float _ppxd, _ppyd;           // principle points
     cv::Mat _Kdepth, _Pdepth;     // camera info matrices

     /** RGB Stream Settings */
     int _cwidth = 848;            // width
     int _cheight = 480;           // height
     int _cfps = 60;               // framerate
     float _fxc, _fyc;             // focal lengths
     float _ppxc, _ppyc;           // principle points
     cv::Mat _Krgb, _Prgb;        // camera info matrices

     /** Misc Cam Params */
     cv::Mat _Dmat;
     float _dscale;
     float _baseline;
     float _trueMinDisparity;
     float _trueMaxDisparity;
     std::string _device_name;
     std::string _distortion_model = "plumb_bob";

     /** Transformation frame ids */
     std::string _camera_base_frame = "camera_link";
     std::string _depth_base_frame = "camera_depth_frame";
     std::string _color_base_frame = "camera_color_frame";
     std::string _depth_optical_frame_id = "camera_depth_optical_frame";
     std::string _color_optical_frame_id = "camera_color_optical_frame";
     std::string _aligned_base_frame = "camera_aligned_depth_to_color_frame";
     const std::string disparity_filter_name = "Disparity";
     const std::string depth_filter_name = "Disparity2Depth";

     /** Private Functions */
     float _get_baseline(bool verbose = false);
     float _get_depth_scale(bool verbose = false);
public:
     /** Constructors */
	CameraD415(bool show_options = false);
	CameraD415(int rgb_fps, int rgb_resolution[2], int depth_fps,
          int depth_resolution[2], bool show_options = false
     );
     ~CameraD415();

     /** Startup - Shutdown - Initialization Functions */
     bool stop();
     void start_thread();
     bool start_streams(std::vector<RS_STREAM_CFG> stream_cfgs);
     bool sensors_startup(std::vector<RS_STREAM_CFG> stream_cfgs);
     bool hardware_startup(std::vector<RS_STREAM_CFG> stream_cfgs);
     bool reset(std::vector<RS_STREAM_CFG> stream_cfgs, bool with_startup = true);

     /** Basic Image Functions */
     rs2::frame get_rgb_frame(bool flag_aligned = false);
     rs2::frame get_depth_frame(bool flag_aligned = false);

     int _get_rgb_image(rs2::frame frame, cv::Mat* image);
     int get_rgb_image(cv::Mat* image, bool flag_aligned = false);

     int _get_depth_image(rs2::frame frame, cv::Mat* image, bool flag_processed = false);
     int get_depth_image(cv::Mat* image, bool flag_aligned = false, bool flag_processed = false);

     int read(cv::Mat* rgb, cv::Mat* depth, cv::Mat* processed, bool flag_aligned = false, bool flag_processed = false);

     /** Frame Post-Processing Functions */
     int process_depth_frame(rs2::frame frame, rs2::frame* processed);
     int process_frames(rs2::frameset frame, rs2::frameset* processed, bool use_filters = false);

     /** Camera Data Conversions */
     cv::Mat convert_to_disparity(const cv::Mat depth, double* conversion_gain, double* conversion_offset);
     /** Experimental */cv::Mat convert_to_disparity_alternative(const cv::Mat depth, double* conversion_gain, double* conversion_offset);
     /** Experimental */cv::Mat convert_to_disparity_test(const cv::Mat depth, double* conversion_gain, double* conversion_offset);
     /**TODO*/int get_pointcloud(const rs2::frame& depth, rs2::points* cloud);
     /**TODO*/int get_pointcloud(const rs2::frame& depth, const rs2::frame& color, rs2::points* cloud);

     /** Device Getters */
     vector<rs2::device> get_available_devices(bool show_features = false, bool verbose = false);
     void get_available_sensors(rs2::device dev);
     void get_sensor_option(const rs2::sensor& sensor);
     std::string get_sensor_name(const rs2::sensor& sensor);
     int get_intrinsics(rs2_stream stream_type, cv::Mat* K, cv::Mat* P, bool verbose = false);
     void get_extrinsics(bool verbose = false);
     float get_baseline(bool verbose = false);
     float get_depth_scale(bool verbose = false);
     std::vector<rs2::filter> get_default_filters(bool use_decimation = false,
          bool use_threshold = true, bool depth_to_disparity = true,
          bool use_spatial = false, bool use_temporal = true, bool use_hole_filling = false
     );

     /** Device Setters */
     void enable_filters();
     void disable_filters();
     void enable_alignment();
     void disable_alignment();
     void enable_timing_debug();
     void disable_timing_debug();

     /** Misc Functions */
     int get_raw_queued_images(cv::Mat* rgb, cv::Mat* depth);
     int get_processed_queued_images(cv::Mat* rgb, cv::Mat* depth);
     int get_processed_queued_images(cv::Mat* rgb, cv::Mat* depth, rs2::points* cloud);
     // int get_queued_images(cv::Mat* rgb, cv::Mat* depth, cv::Mat* disparity, bool get_disparity = true);

     void processingThread();
};

#endif /* CAMERA_D415_H_*/
