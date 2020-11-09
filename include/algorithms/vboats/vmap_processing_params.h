#ifndef VBOATS_VMAP_PROCESSING_PARAMS_H_
#define VBOATS_VMAP_PROCESSING_PARAMS_H_

#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>

class VmapProcessingParams{
public:
     VmapProcessingParams(){};

     // Stripping-Based Filtering Params
     std::vector<float> stripping_threshs              = {0.3, 0.3, 0.25, 0.4};
     // Sobelized-Based Filtering Params
     std::vector<int> sobel_kernel_multipliers         = {1, 1};
     int sobel_preprocessing_thresh_sobel              = 35;
     int sobel_preprocessing_blur_size                 = 5;
     int sobel_postprocessing_thresh_prefiltering      = 40;
     int sobel_postprocessing_thresh_postfiltering     = 15;
     int sobel_postprocessing_blur_size                = 7;
     // Ground Line Searching Params
     double gnd_line_search_min_deg                    = 26.0;
     double gnd_line_search_max_deg                    = 89.0;
     double gnd_line_search_deadzone                   = 2.0;
     int gnd_line_search_hough_thresh                  = 100;
     // V-Map Dependent Depth Filtering Params
     int depth_filtering_gnd_line_intercept_offset     = 0;

     // Setters
     void set_stripping_thresholds(std::vector<float> values){
          this->stripping_threshs.clear();
          this->stripping_threshs.assign(values.begin(), values.end());
     }
     void set_sobelize_kernel_multipliers(int x, int y){
          this->sobel_kernel_multipliers.at(0) = x;
          this->sobel_kernel_multipliers.at(1) = y;
     }
     void set_threshold_before_sobelizing(int value){ this->sobel_preprocessing_thresh_sobel = value; }
     void set_threshold_sobelize_postprocessing_prefiltering(int value){ this->sobel_postprocessing_thresh_prefiltering = value; }
     void set_threshold_sobelize_postprocessing_postfiltering(int value){ this->sobel_postprocessing_thresh_postfiltering = value; }
     void set_sobelize_preprocessing_blur_size(int value){ this->sobel_preprocessing_blur_size = value; }
     void set_sobelize_postprocessing_blur_size(int value){ this->sobel_postprocessing_blur_size = value; }

     void set_ground_line_search_hough_threshold(int value){ this->gnd_line_search_hough_thresh = value; }
     void set_ground_line_search_min_angle(double value){ this->gnd_line_search_min_deg = value; }
     void set_ground_line_search_max_angle(double value){ this->gnd_line_search_max_deg = value; }
     void set_ground_line_search_deadzone(double value){ this->gnd_line_search_deadzone = value; }
     void set_depth_filtering_ground_line_intercepty_offset(int value){ this->depth_filtering_gnd_line_intercept_offset = value; }
};

class VmapProcessingDebugObjects{
public:
    VmapProcessingDebugObjects(){};

    cv::Mat preprocessing_keep_mask;
    void update(bool depth_based, float cam_baseline = 0, float cam_dscale = 0){}
};

#endif // VBOATS_VMAP_PROCESSING_PARAMS_H_
