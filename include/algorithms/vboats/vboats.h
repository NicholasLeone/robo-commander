#ifndef ROBOCOMMANDER_ALGORITHMS_VBOATS_H_
#define ROBOCOMMANDER_ALGORITHMS_VBOATS_H_

#include <string>
#include <vector>

#include "base/definitions.h"
#include "utilities/image_utils.h"
#include "algorithms/vboats/obstacle.h"
#include "algorithms/vboats/umap_processing_params.h"
#include "algorithms/vboats/vmap_processing_params.h"
#include "algorithms/vboats/vboats_processing_images.h"
#include "algorithms/vboats/vboats_utils.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// #ifdef WITH_CUDA
	#include <memory>
	#include <opencv2/core/cuda.hpp>
// #endif

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> cloudxyz_t;

typedef enum ContourFilterMethod{
	PERIMETER_BASED = 1,
	AREA_BASED = 2,
}ContourFilterMethod;
typedef enum UvMapFilterMethod{
	STRIPPING_METHOD = 0,
	SOBELIZED_METHOD = 1,
}UvMapFilterMethod;
typedef enum ImageAngleCorrectionType{
	ROLL_CORRECTION = 0,
	PITCH_CORRECTION = 1,
	YAW_CORRECTION = 2,
}ImageAngleCorrectionType;

class Vboats{
public:
	UmapProcessingParams umapParams;
	VmapProcessingParams vmapParams;
	VboatsProcessingImages processingDebugger;
	BufferUmapProcessing umapBuffer;
	BufferVmapProcessing vmapBuffer;
	// #ifdef WITH_CUDA
	// #endif
public:
	/** Constructors */
	Vboats();
	~Vboats();

	/** Startup - Shutdown - Initialization Functions */
	void init();
	void update();

	cv::Mat remove_umap_deadzones(const cv::Mat& umap);
	cv::Mat remove_vmap_deadzones(const cv::Mat& vmap);
	cv::Mat generate_disparity_from_depth(const cv::Mat& depth);
	cloudxyz_t::Ptr generate_pointcloud_from_depth(const cv::Mat& depth, bool debug_timing = false);

	int process(const cv::Mat& depth, cv::Mat* filtered_input,
		std::vector<Obstacle>* found_obstacles = nullptr,
		std::vector<float>* line_coefficients = nullptr,
		cv::Mat* disparity_output = nullptr,
		cv::Mat* umap_output = nullptr, cv::Mat* vmap_output = nullptr,
		cv::Mat* umap_input = nullptr, cv::Mat* vmap_input = nullptr,
		bool verbose_obstacles = false, bool debug = false
	);

	int process_w_cuda(const cv::Mat& depth, cv::Mat* filtered_input,
		std::vector<Obstacle>* found_obstacles = nullptr,
		std::vector<float>* line_coefficients = nullptr,
		cv::Mat* disparity_output = nullptr,
		cv::Mat* umap_output = nullptr, cv::Mat* vmap_output = nullptr,
		cv::Mat* umap_input = nullptr, cv::Mat* vmap_input = nullptr,
		bool verbose_obstacles = false, bool debug = false
	);
	// std::shared_ptr<std::vector<cv::Mat>> computeArray(
	//      std::shared_ptr<std::vector< cv::cuda::HostMem >> srcMemArray,
	//      std::shared_ptr<std::vector< cv::cuda::HostMem >> dstMemArray,
	//      std::shared_ptr<std::vector< cv::cuda::GpuMat >> gpuSrcArray,
	//      std::shared_ptr<std::vector< cv::cuda::GpuMat >> gpuDstArray,
	//      std::shared_ptr<std::vector< cv::Mat >> outArray,
	//      std::shared_ptr<std::vector< cv::cuda::Stream >> streamsArray
	// );

	// Config Setters
	void set_contour_filtering_method(std::string method);
	void set_umap_processing_method(std::string method);
	void set_vmap_processing_method(std::string method);
	void set_image_angle_correction_type(std::string method);

	// Runtime Togglers
	void enable_angle_correction(bool flag = true);
	void enable_correction_angle_sign_flip(bool flag = true);
	void enable_filtered_depth_denoising(bool flag = true);
	void enable_obstacle_data_extraction(bool flag = true);
	void enable_noisy_gnd_line_filtering(bool flag = true);
	void enable_image_processing_timings_debug(bool flag = true);
	void toggle_disparity_generation_debug_verbosity(bool flag = true);

	// Runtime Setters
	void set_camera_info(cv::Mat K, float depth_scale, float baseline, bool verbose = false);
	void set_camera_info(float fx, float fy, float px, float py, float depth_scale, float baseline, bool verbose = false);
	void set_camera_orientation(double roll, double pitch, double yaw, bool verbose = false);
	void set_camera_orientation(double x, double y, double z, double w, bool verbose = false);
	void set_camera_angle_offset(double offset_degrees);
	void set_depth_denoising_kernel_size(int size);
	void set_absolute_minimum_depth(float value);
	void set_absolute_maximum_depth(float value);
	void set_object_dimension_limits_x(float min, float max);
	void set_object_dimension_limits_y(float min, float max);
	void flip_object_dimension_x_limits(bool flag = false);
	void flip_object_dimension_y_limits(bool flag = false);
	void set_gnd_line_slope_error_threshold(float value);
	void set_gnd_line_intercept_error_threshold(int value);
	// Getters
	bool is_obstacle_data_extraction_performed();
	bool is_depth_denoising_performed();
	bool is_angle_correction_performed();
	double get_depth_absolute_min();
	double get_depth_absolute_max();
	double get_correction_angle(bool in_degrees = false, bool flip_sign = false);
	ImageAngleCorrectionType get_angle_correction_type();
	std::vector<double> get_camera_angles();

private:
	std::string classLbl = txt_bold_magenta() + "Vboats" + txt_reset_color();
	// Counters
	int _cam_info_count = 0;

	// TODO: Section Name
	float _hard_min_depth = 0.01;
	float _hard_max_depth = 20.0;
	float _object_min_dimension_x = 0.0;
	float _object_max_dimension_x = 0.0;
	float _object_min_dimension_y = 0.0;
	float _object_max_dimension_y = 0.0;
	bool _flip_object_dimension_x_limits = false;
	bool _flip_object_dimension_y_limits = false;

	// TODO: Section Name
	float _fx                     = 0;
	float _fy                     = 0;
	float _px                     = 0;
	float _py                     = 0;
	float _baseline               = 0;
	float _depth_scale            = 0;
	float _Tx                     = 0;
	float _depth2disparityFactor  = 0;
	float _depth_deproject_gain   = 0;

	// TODO: Section Name
	double _cam_roll    = 0.0;
	double _cam_pitch   = 0.0;
	double _cam_yaw     = 0.0;
	double _cam_angle_offset     = 0.0;
	int _filtered_depth_denoising_size = 2;

	// TODO: Section Name
	float _prev_gnd_line_slope    		= 0.0;
	float _delta_gnd_line_slope_thresh		= 1.5;
	int _prev_gnd_line_intercept     		= 0;
	int _delta_gnd_line_intercept_thresh	= 300;

	// TODO: Section Name
	ContourFilterMethod _contourFiltMeth = PERIMETER_BASED;
	UvMapFilterMethod _umapFiltMeth = SOBELIZED_METHOD;
	UvMapFilterMethod _vmapFiltMeth = SOBELIZED_METHOD;
	ImageAngleCorrectionType _angleCorrectionType = ROLL_CORRECTION;

	// Operational Flags
	bool _have_cam_info = false;
	bool _is_ground_present = true;
	bool _do_angle_correction = true;
	bool _denoise_filtered_depth = true;
	bool _do_obstacle_data_extraction = true;
	bool _angle_correction_performed = false;
	bool _flip_correction_angle_sign = false;
	bool _check_gnd_line_noise = false;

	// Debug Objects
	bool _debug_disparity_gen = false;
	bool _debug_process_timings = false;

	#ifdef WITH_CUDA
	// cv::cuda::Stream _umapCudaStr;
	// cv::cuda::Stream _vmapCudaStr;
	// std::shared_ptr< std::vector<cv::cuda::Stream> > _cudaSreams;
	#endif
};

#endif // ROBOCOMMANDER_ALGORITHMS_VBOATS_H_
