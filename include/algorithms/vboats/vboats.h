#ifndef VBOATS_VBOATS_H_
#define VBOATS_VBOATS_H_

#include <string>
#include <chrono>
#include <vector>
#include <thread>
#include <mutex>

#include <opencv2/opencv.hpp>

using namespace std;

class VBOATS{
private:
     // Counters
     int nObs = 0;
     // Operational Flags
     bool is_ground_present = true;

     // Debug Flags
     bool debug = false;
     bool flag_simulation = false;
     bool debug_contours = false;
     bool debug_windows = false;
     bool debug_obstacle_search = false;
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

     float get_uv_map(const cv::Mat& image, cv::Mat* umap, cv::Mat* vmap,
          bool visualize = false, bool verbose = false, bool timing = false);

     /** TODO: Convert these python functions to C++ */
     // def get_uv_map(self, img, verbose=False, timing=False)
     // def extract_contour_bounds(self, cnts, verbose=False, timing=False)
     // def find_contours(self, _umap, threshold = 30.0, threshold_method = "perimeter", offset=(0,0), max_thresh=1500.0, debug=False)
     // def find_obstacles(self, vmap, dLims, xLims, search_thresholds = (3,30), ground_detected=True, verbose=False,timing=False)
     // def obstacle_search(self, _vmap, x_limits, pixel_thresholds=(1,30), window_size=None, verbose=False, timing=False)
     // def find_obstacles_disparity(self, vmap, dLims, xLims, search_thresholds = (3,30), ground_detected=True, lineCoeffs=None, verbose=False,timing=False)
     // def obstacle_search_disparity(self, _vmap, x_limits, pixel_thresholds=(1,30), window_size=None, lineCoeffs=None, verbose=False, timing=False)
     // def get_vmap_mask(self, vmap, threshold=20, min_ground_pixels=6, shift_gain=2, dxmean_thresh=1.0, max_extensions=3, extension = 4, maxStep=14, deltas=(0,20), mask_size = [10,30], window_size = [10,30], draw_method=1, verbose=False, timing=False)
     // def is_gnd_present_disparity(self,found_lines,minDeg=-89.0, maxDeg=-26.0)
     // def estimate_houghline_coeffs(self, filtered_vmap,hough_thresh=100, deg_offset=2.0)
     //
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
