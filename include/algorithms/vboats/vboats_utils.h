#ifndef VBOATS_VBOATS_UTILS_H_
#define VBOATS_VBOATS_UTILS_H_

#include <vector>
#include <string.h>
#include <algorithm>          // for copy() and assign()

#include "base/definitions.h"
#include "utilities/utils.h"
#include "utilities/image_utils.h"
#include "algorithms/vboats/obstacle.h"
#include "algorithms/vboats/umap_processing_params.h"
#include "algorithms/vboats/vmap_processing_params.h"
#include "algorithms/vboats/vboats_processing_images.h"

#ifdef WITH_CUDA
#include <opencv2/core/cuda.hpp>

struct BufferUmapProcessing{
     cv::cuda::GpuMat inputGpu;
     cv::cuda::HostMem inputCpu;
     cv::cuda::GpuMat outputGpu;
     cv::cuda::HostMem outputCpu;
     cv::cuda::GpuMat umapSobel;
     cv::cuda::GpuMat sobelThreshed;
     cv::cuda::GpuMat sobelDilated;
     cv::cuda::GpuMat sobelBlurred;
     cv::cuda::GpuMat keepMask;
     cv::cuda::GpuMat rawUmap;
     cv::cuda::GpuMat umapThreshed;
     cv::cuda::Stream stream;
     cv::Mat processed;
};
struct BufferVmapProcessing{
     cv::cuda::GpuMat inputGpu;
     cv::cuda::GpuMat outputGpu;
     cv::cuda::HostMem inputCpu;
     cv::cuda::HostMem preoutputCpu;
     cv::cuda::HostMem outputCpu;
     cv::cuda::GpuMat rawVmap;
     cv::cuda::GpuMat blurVmap;
     cv::cuda::GpuMat sobel;
     cv::cuda::GpuMat preprocessed_sobel;
     cv::cuda::GpuMat sobelThreshed;
     cv::cuda::GpuMat sobelBlurred;
     cv::cuda::GpuMat keepMask;
     cv::cuda::Stream stream;
     cv::Mat preprocessed;
     cv::Mat postprocessed;
};

int process_uvmaps_sobelized_cuda(const cv::Mat& umap, const cv::Mat& vmap,
     UmapProcessingParams umapParams, VmapProcessingParams vmapParams,
     const BufferUmapProcessing& bu, const BufferVmapProcessing& bv
);
#endif

using namespace std;

std::vector<double> quaternion_to_euler(double qx, double qy, double qz, double qw);

/** UV-Map Generation */
void genUVMap(cv::Mat image, cv::Mat* umap, cv::Mat* vmap, bool verbose = false);
void genUVMapThreaded(cv::Mat image, cv::Mat* umap, cv::Mat* vmap, double nThreads = -1.0, bool verbose = false);

/** UV-Map Processing */
cv::Mat preprocess_umap_stripping(const cv::Mat& input,
     vector<float>* thresholds, bool verbose = false, bool debug = false
);
void preprocess_umap_stripping(const cv::Mat& input, cv::Mat* output,
     vector<float>* thresholds, bool verbose = false, bool debug = false
);
cv::Mat preprocess_vmap_stripping(const cv::Mat& input, vector<float>* thresholds,
     bool verbose = false, bool debug = false
);
void preprocess_vmap_stripping(const cv::Mat& input, cv::Mat* output,
     vector<float>* thresholds, bool verbose = false, bool debug = false
);
cv::Mat preprocess_umap_sobelized(const cv::Mat& umap, int thresh_pre_sobel = 15,
     int thresh_sobel_preprocess = 10, int thresh_sobel_postprocess = 2,
     int dilate_size = 1, int blur_size = 3, vector<int> kernel_multipliers = {},
     cv::Mat* keep_mask = nullptr, cv::Mat* sobel_raw = nullptr, cv::Mat* sobel_preprocessed = nullptr,
     cv::Mat* sobel_dilated = nullptr, cv::Mat* sobel_blurred = nullptr
);
cv::Mat preprocess_umap_sobelized(const cv::Mat& umap, int thresh_pre_sobel = 15,
     int thresh_sobel_preprocess = 10, int thresh_sobel_postprocess = 2,
     int dilate_size = 1, int blur_size = 3, vector<int> kernel_multipliers = {},
     VboatsProcessingImages* image_debugger = nullptr
);
cv::Mat preprocess_vmap_sobelized(const cv::Mat& vmap,
     int thresh_sobel = 35, int blur_size = 5, vector<int> kernel_multipliers = {}
);
cv::Mat postprocess_vmap_sobelized(const cv::Mat& vmap, const cv::Mat& preprocessed_sobel,
     int thresh_preprocess = 40, int thresh_postprocess = 15, int blur_size = 7,
     vector<int> kernel_multipliers = {}, cv::Mat* sobel_threshed = nullptr,
     cv::Mat* sobel_blurred = nullptr, cv::Mat* keep_mask = nullptr
);
cv::Mat postprocess_vmap_sobelized(const cv::Mat& vmap, const cv::Mat& preprocessed_sobel,
     int thresh_preprocess = 40, int thresh_postprocess = 15, int blur_size = 7,
     vector<int> kernel_multipliers = {}, VboatsProcessingImages* image_debugger = nullptr
);

/** Ground Line Extraction */
void get_hough_line_params(const float& rho, const float& theta, float* slope, int* intercept);
int estimate_ground_line_coefficients(const vector<cv::Vec2f>& lines,
     float* best_slope, int* best_intercept, float* worst_slope, int* worst_intercept,
     double gnd_deadzone = 2.0, double minDeg = 26.0, double maxDeg = 89.0,
     bool verbose = false, bool debug_timing = false
);
int find_ground_lines(const cv::Mat& vmap, cv::Mat* rhos, cv::Mat* thetas, int hough_thresh = 100, bool verbose = false);
int find_ground_lines(const cv::Mat& vmap, cv::Mat* found_lines, int hough_thresh = 100, bool verbose = false);
int find_ground_lines(const cv::Mat& vmap, vector<cv::Vec2f>* found_lines, int hough_thresh = 100, bool verbose = false);
bool find_ground_line(const cv::Mat& vmap, std::vector<float>* best_coeffs,
     double minDeg = 26.0, double maxDeg = 89.0, double gnd_deadzone = 2.0, int hough_thresh = 100,
     bool verbose = false, bool debug_timing = false
);
bool find_ground_line(const cv::Mat& vmap, float* best_slope, int* best_intercept,
     double minDeg = 26.0, double maxDeg = 89.0, double gnd_deadzone = 2.0, int hough_thresh = 100,
     bool verbose = false, bool debug_timing = false
);

/** U-Map Contour Extraction */
void extract_contour_bounds(const vector<cv::Point>& contour, vector<int>* xbounds, vector<int>* dbounds, bool verbose = false);
void find_contours(const cv::Mat& umap, vector<vector<cv::Point>>* filtered_contours,
     int filter_method = 1, float min_threshold = 30.0, float max_threshold = -1,
     cv::Point* offset = nullptr, vector<cv::Vec4i>* filtered_hierarchies = nullptr,
     vector<vector<cv::Point>>* all_contours = nullptr, vector<cv::Vec4i>* all_hierarchies = nullptr,
     bool verbose = false, bool debug = false, bool debug_timing = false
);

/** Obstacle Extraction */
int obstacle_search_disparity(const cv::Mat& vmap, const vector<int>& xLimits, vector<int>* yLimits,
     int* pixel_thresholds = nullptr, int* window_size = nullptr,
     std::vector<float> line_params = {}, vector<cv::Rect>* obs_windows = nullptr,
     bool verbose = true, bool debug = false, bool debug_timing = false
);
int find_obstacles_disparity(const cv::Mat& vmap,
     const std::vector< std::vector<cv::Point> >& contours,
     std::vector<float> line_params, std::vector<Obstacle>* found_obstacles,
     std::vector< std::vector<cv::Rect> >* obstacle_windows = nullptr,
     bool verbose = false, bool debug_timing = false
);
int find_obstacles_disparity(const cv::Mat& vmap,
     const std::vector< std::vector<cv::Point> >& contours,
     std::vector<float> line_params, std::vector<Obstacle>* found_obstacles,
     VboatsProcessingImages* image_debugger = nullptr,
     bool verbose = false, bool debug_timing = false
);

/** Depth Filtering Strategies */

/** Filter out any pixels in a depth image that are at or below the ground. This is
* done via a "keep" mask created by removing pixels in the corresponding disparity
* image that are located at or below the estimated ground line found in the v-map.
*/
// int filter_depth_using_ground_line(const cv::Mat& depth, const cv::Mat& disparity,
//      const cv::Mat& vmap, std::vector<float> line_params, cv::Mat* filtered_image,
//      std::vector<int> line_intercept_offsets = {}, cv::Mat* keep_mask = nullptr,
//      bool verbose = false, bool debug_timing = false
// );
// int filter_depth_using_ground_line(const cv::Mat& depth, const cv::Mat& disparity,
//      const cv::Mat& vmap, std::vector<float> line_params, cv::Mat* filtered_image,
//      std::vector<int> line_intercept_offsets = {}, VboatsProcessingImages* image_debugger = nullptr,
//      bool verbose = false, bool debug_timing = false
// );
int filter_depth_using_ground_line(const cv::Mat& depth, const cv::Mat& disparity,
     const cv::Mat& vmap, std::vector<float> line_params, cv::Mat* filtered_image,
     int line_intercept_offset = 0, cv::Mat* keep_mask = nullptr, cv::Mat* keep_mask_vmap = nullptr,
     bool verbose = false, bool debug_timing = false
);
int filter_depth_using_ground_line(const cv::Mat& depth, const cv::Mat& disparity,
     const cv::Mat& vmap, std::vector<float> line_params, cv::Mat* filtered_image,
     int line_intercept_offset = 0, VboatsProcessingImages* image_debugger = nullptr,
     bool verbose = false, bool debug_timing = false
);
int filter_depth_using_object_candidate_regions(const cv::Mat& depth, const cv::Mat& disparity,
     const cv::Mat& vmap, const vector<vector<cv::Point>>& contours, cv::Mat* filtered_image,
     std::vector<float> line_params, int gnd_line_offset = 0, cv::Mat* keep_mask = nullptr,
     cv::Mat* vmap_objects = nullptr, std::vector<cv::Rect>* vmap_search_regions = nullptr,
     bool verbose = false, bool debug = false, bool debug_img_info = false, bool debug_timing = false
);
int filter_depth_using_object_candidate_regions(const cv::Mat& depth, const cv::Mat& disparity,
     const cv::Mat& vmap, const vector<vector<cv::Point>>& contours, cv::Mat* filtered_image,
     std::vector<float> line_params, int gnd_line_offset = 0, VboatsProcessingImages* image_debugger = nullptr,
     bool verbose = false, bool debug = false, bool debug_img_info = false, bool debug_timing = false
);

#endif // VBOATS_VBOATS_UTILS_H_
