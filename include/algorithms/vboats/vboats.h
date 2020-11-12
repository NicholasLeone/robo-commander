#ifndef ROBOCOMMANDER_ALGORITHMS_VBOATS_H_
#define ROBOCOMMANDER_ALGORITHMS_VBOATS_H_

#include <string>
#include <vector>

#include "base/definitions.h"
#include "utilities/image_utils.h"
#include "algorithms/vboats/umap_processing_params.h"
#include "algorithms/vboats/vmap_processing_params.h"
// #include "algorithms/vboats/image_processing_debugging.h"

using namespace std;

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

public:
	/** Constructors */
	Vboats();
	~Vboats();

	/** Startup - Shutdown - Initialization Functions */
	void init();
	void update();
	int process(const cv::Mat& depth);

	cv::Mat remove_umap_deadzones(const cv::Mat& umap);
	cv::Mat remove_vmap_deadzones(const cv::Mat& vmap);
	cv::Mat generate_disparity_from_depth(const cv::Mat& depth);

	void set_camera_info(cv::Mat K, float depth_scale, float baseline, bool verbose = false);
	void set_camera_orientation(double roll, double pitch, double yaw, bool verbose = false);
	void set_camera_orientation(double x, double y, double z, double w, bool verbose = false);
	void set_camera_angle_offset(double offset_degrees);
	void set_depth_denoising_kernel_size(int size);

	void set_absolute_minimum_depth(float value);
	void set_absolute_maximum_depth(float value);
	void set_contour_filtering_method(std::string method);
	void set_umap_processing_method(std::string method);
	void set_vmap_processing_method(std::string method);
	void set_image_angle_correction_type(std::string method);

	void enable_angle_correction(bool flag = true);
	void enable_filtered_depth_denoising(bool flag = true);
	void enable_obstacle_data_extraction(bool flag = true);

private:
	std::string classLbl = txt_bold_magenta() + "Vboats" + txt_reset_color();
	// Counters
	int _cam_info_count = 0;

	// TODO: Section Name
	float _hard_min_depth = 0.01;
	float _hard_max_depth = 20.0;

	// TODO: Section Name
	float _fx                     = 0;
	float _fy                     = 0;
	float _px                     = 0;
	float _py                     = 0;
	float _baseline               = 0;
	float _depth_scale            = 0;
	float _Tx                     = 0;
	float _depth2disparityFactor  = 0;

	// TODO: Section Name
	double _cam_roll    = 0.0;
	double _cam_pitch   = 0.0;
	double _cam_yaw     = 0.0;
	double _cam_angle_offset     = 0.0;
	int _filtered_depth_denoising_size = 2;

	// TODO: Section Name
	cv::Mat _cur_depth;
	cv::Mat _cur_disparity;
	cv::Mat _cur_umap;
	cv::Mat _cur_vmap;

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

	// Debug Objects
	bool _verbose = false;
	UmapProcessingDebugObjects _umap_debugger;
	VmapProcessingDebugObjects _vmap_debugger;
};

#endif // ROBOCOMMANDER_ALGORITHMS_VBOATS_H_
