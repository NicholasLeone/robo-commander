#ifndef ROBOCOMMANDER_ALGORITHMS_VBOATS_H_
#define ROBOCOMMANDER_ALGORITHMS_VBOATS_H_

#include <string>
#include <chrono>
#include <vector>
#include <thread>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <opencv2/core/version.hpp> /* For OpenCV backwards compatibility */

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
         float dtype_gain = 0, float aux_dist_factor = 0, bool verbose = true
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
     void update();

     void filter_disparity_vmap(const cv::Mat& input, cv::Mat* output, vector<float>* thresholds, bool verbose = false, bool debug = false, bool visualize = false);
     void filter_disparity_umap(const cv::Mat& input, cv::Mat* output, vector<float>* thresholds, bool verbose = false, bool debug = false, bool visualize = false);

     bool is_ground_present();
     int find_ground_lines(const cv::Mat& vmap, cv::Mat* rhos, cv::Mat* thetas, int hough_thresh = 100, bool verbose = false);
     int find_ground_lines(const cv::Mat& vmap, cv::Mat* found_lines, int hough_thresh = 100, bool verbose = false);
     int find_ground_lines(const cv::Mat& vmap, vector<cv::Vec2f>* found_lines, int hough_thresh = 100, bool verbose = false);
     void get_hough_line_params(const float& rho, const float& theta, float* slope, int* intercept);
     int estimate_ground_line(const vector<cv::Vec2f>& lines, float* best_slope, int* best_intercept, float* worst_slope, int* worst_intercept,
          double gnd_deadzone = 2.0, double minDeg = 26.0, double maxDeg = 89.0, bool verbose = false, bool debug_timing = false
     );
     bool find_ground_line(const cv::Mat& vmap, float* best_slope, int* best_intercept,
          double minDeg = 26.0, double maxDeg = 89.0, int hough_thresh = 100,
          double gnd_deadzone = 2.0, bool verbose = false, bool debug_timing = false, bool visualize = false
     );

     void extract_contour_bounds(const vector<cv::Point>& contour, vector<int>* xbounds, vector<int>* dbounds, bool verbose = false);
     void find_contours(const cv::Mat& umap, vector<vector<cv::Point>>* found_contours,
          int filter_method = 1, float min_threshold = 30.0, int* offsets = nullptr,
          float max_threshold = -1, bool verbose = false, bool visualize = false,
          bool debug = false, bool debug_timing = false
     );

     int obstacle_search_disparity(const cv::Mat& vmap, const vector<int>& xLimits, vector<int>* yLimits,
          int* pixel_thresholds = nullptr, int* window_size = nullptr,
          std::vector<float> line_params = {}, vector<cv::Rect>* obs_windows = nullptr,
          bool verbose = true, bool visualize = false, bool debug = false, bool debug_timing = false
     );
     int find_obstacles_disparity(const cv::Mat& vmap, const vector<vector<cv::Point>>& contours,
           vector<Obstacle>* found_obstacles, std::vector<float> line_params, vector< vector<cv::Rect> >* obstacle_windows = nullptr,
           bool verbose = false, bool debug_timing = false
     );

     int pipeline_disparity(const cv::Mat& disparity, const cv::Mat& umap, const cv::Mat& vmap,
          vector<Obstacle>* obstacles, cv::Mat* uMorphElement = nullptr,
          bool verbose = false, bool debug_timing = false, bool visualize = false
     );
};

#endif // ROBOCOMMANDER_ALGORITHMS_VBOATS_H_
