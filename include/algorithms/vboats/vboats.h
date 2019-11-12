#ifndef VBOATS_VBOATS_H_
#define VBOATS_VBOATS_H_

#include <string>
#include <chrono>
#include <vector>
#include <thread>
#include <mutex>

#include <opencv2/opencv.hpp>

using namespace std;

class Obstacle{
public:
    Obstacle(vector<cv::Point> pts, vector<int> dBounds);
    int dMin;
    int dMax;
    cv::Point maxXY;
    cv::Point minXY;
    double angle;
    float distance;
    cv::Point3f location;
    void update(bool depth_based, float cam_baseline = 0, float cam_dscale = 0,
         float* cam_focal = nullptr, float* cam_principal_point = nullptr,
         float dtype_gain = 0, float aux_dist_factor = 0, bool verbose = false
    );
};

class VBOATS{
private:
     // Counters
     int nObs = 0;
     // Operational Flags
     bool _is_ground_present = true;

     // Debug Flags
     bool _debug = false;
     bool _flag_simulation = false;
     bool _debug_contours = false;
     bool _debug_windows = false;
     bool _debug_obstacle_search = false;
public:
     /** Constructors */
     VBOATS();
     ~VBOATS();

     /** Startup - Shutdown - Initialization Functions */
     void init();

     /**
     * Updates the locally stored observations for use in next prediction step
     *
     *    @param todo: newest image
     */
     void update();

     float get_uv_map(cv::Mat image, cv::Mat* umap, cv::Mat* vmap,
          bool visualize = false, std::string dispId = "", bool verbose = true,
          bool debug = false, bool timing = false
     );

     /** WARNING: Experimental function */
     float get_uv_map_scaled(cv::Mat image, cv::Mat* umap, cv::Mat* vmap, double scale,
          bool visualize = false, std::string dispId = "", bool verbose = false,
          bool debug = false, bool timing = false
     );

     void filter_disparity_vmap(const cv::Mat& input, cv::Mat* output, vector<float>* thresholds, bool verbose = false, bool visualize = false);
     void filter_disparity_umap(const cv::Mat& input, cv::Mat* output, vector<float>* thresholds, bool verbose = false, bool visualize = false);

     int find_ground_lines(const cv::Mat& vmap, cv::Mat* rhos, cv::Mat* thetas, int hough_thresh = 100, bool verbose = false);
     int find_ground_lines(const cv::Mat& vmap, cv::Mat* found_lines, int hough_thresh = 100, bool verbose = false);
     int find_ground_lines(const cv::Mat& vmap, vector<cv::Vec2f>* found_lines, int hough_thresh = 100, bool verbose = false);

     void get_hough_line_params(const float& rho, const float& theta, float* slope, int* intercept);

     int estimate_ground_line(const vector<cv::Vec2f>& lines, float* best_slope, int* best_intercept, float* worst_slope, int* worst_intercept,
          double gnd_deadzone = 2.0, double minDeg = 26.0, double maxDeg = 89.0, bool verbose = false, bool debug_timing = false
     );
     bool is_ground_present(const cv::Mat& vmap, float* best_slope, int* best_intercept,
          int hough_thresh = 100, double gnd_deadzone = 2.0, double minDeg = 26.0,
          double maxDeg = 89.0, bool verbose = false, bool debug_timing = false, bool visualize = false
     );

     void find_contours(const cv::Mat& umap, vector<vector<cv::Point>>* found_contours,
          int filter_method = 1, float min_threshold = 30.0, int* offsets = nullptr,
          float max_threshold = -1, bool verbose = false, bool visualize = false,
          bool debug = false, bool debug_timing = false
     );

     void extract_contour_bounds(const vector<cv::Point>& contour, vector<int>* xbounds, vector<int>* dbounds, bool verbose = false);

     int obstacle_search_disparity(const cv::Mat& vmap, const vector<int>& xLimits, vector<int>* yLimits,
          int* pixel_thresholds = nullptr, int* window_size = nullptr, float* line_params = nullptr,
          bool verbose = true, bool visualize = false, bool debug = false, bool debug_timing = false
     );

     int find_obstacles_disparity(const cv::Mat& vmap, const vector<vector<cv::Point>>& contours,
           vector<Obstacle>* found_obstacles, float* line_params, bool verbose = false, bool debug_timing = false
     );

     void pipeline_disparity(const cv::Mat& disparity, const cv::Mat& umap, const cv::Mat& vmap,
          vector<Obstacle>* obstacles, cv::Mat* uMorphElement = nullptr, bool verbose = false, bool debug_timing = false
     );

     /** TODO: Convert these python functions to C++ */
     // def find_obstacles(self, vmap, dLims, xLims, search_thresholds = (3,30), ground_detected=True, verbose=False,timing=False)
     // def obstacle_search(self, _vmap, x_limits, pixel_thresholds=(1,30), window_size=None, verbose=False, timing=False)
     // def get_vmap_mask(self, vmap, threshold=20, min_ground_pixels=6, shift_gain=2, dxmean_thresh=1.0, max_extensions=3, extension = 4, maxStep=14, deltas=(0,20), mask_size = [10,30], window_size = [10,30], draw_method=1, verbose=False, timing=False)

     // def calculate_distance(self, umap, xs, ds, ys, focal=[462.138,462.138],
     //    baseline=0.055, dscale=0.001, pp=[320.551,232.202], dsbuffer=1,
     //    use_principle_point=True, use_disparity_buffer=True, verbose=False)
     //
     // def calculate_distance_disparity(self, umap, xs, ds, ys, focal=[462.138,462.138],
     //  baseline=0.055, dscale=0.001, pp=[320.551,232.202], dsbuffer=1, aux_dist_factor=1.35, dtype_cvt_gain=None,
     //  use_principle_point=True, use_disparity_buffer=True, verbose=False)
     //
     // def extract_obstacle_information(self, minDistance=0.05, verbose=True)
     // def extract_obstacle_information_disparity(self, umap, xBounds, dBounds, obstacles, focal=[462.138,462.138],
     //    baseline=0.055, dscale=0.001, pp=[320.551,232.202], dtype_cvt_gain=None, verbose=True)
     //
     // def pipelineV1(self,_img, timing=False,debug_timing=False)
     // def pipelineV0(self, _img, threshU1=7, threshU2=20,threshV2=70, timing=False)
     // def pipelineTest(self, _img, threshU1=0.25, threshU2=0.3, threshV1=5, threshV2=70, timing=False)

     // TODO: Helper Functions to port
     // def umap_displays(self, border_color=(255,0,255))
     // def vmap_displays(self, border_color=(255,0,255))
     // def generate_visualization(self, dists, angs, flip_ratio=False,use_rgb=True, alpha=0.35,font_scale = 0.35,verbose=False)

};

#endif // VBOATS_VBOATS_H_
