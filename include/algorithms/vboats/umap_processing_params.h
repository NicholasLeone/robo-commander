#ifndef VBOATS_UMAP_PROCESSING_PARAMS_H_
#define VBOATS_UMAP_PROCESSING_PARAMS_H_

#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>

class UmapProcessingParams{
public:
     UmapProcessingParams() : contour_filtering_offset(0, 0){}

     // Contour Filtering
     cv::Point contour_filtering_offset;
     float contour_filtering_thresh_min = 40.0;
     float contour_filtering_thresh_max = -1.0;

     // Stripping-Based Filtering Params
     std::vector<float> stripping_threshs = {0.3, 0.295, 0.275, 0.3};
     // Sobelized-Based Filtering Params
     std::vector<int> sobel_kernel_multipliers = {10, 2};
     int sobel_thresh_pre_sobel         = 15;
     int sobel_thresh_sobel_preprocess  = 10;
     int sobel_thresh_sobel_postprocess = 2;
     int sobel_dilate_size              = 1;
     int sobel_blur_size                = 3;

     // Setters
     void set_contour_filtering_threshold_min(float value){ this->contour_filtering_thresh_min = value; }
     void set_contour_filtering_threshold_max(float value){ this->contour_filtering_thresh_max = value; }

     void set_threshold_before_sobelizing(int value){ this->sobel_thresh_pre_sobel = value; }
     void set_threshold_sobelize_preprocessing(int value){ this->sobel_thresh_sobel_preprocess = value; }
     void set_threshold_sobelize_postprocessing(int value){ this->sobel_thresh_sobel_postprocess = value; }
     void set_sobelize_dilate_size(int value){ this->sobel_dilate_size = value; }
     void set_sobelize_blur_size(int value){ this->sobel_blur_size = value; }

     void set_stripping_thresholds(std::vector<float> values){
          this->stripping_threshs.clear();
          this->stripping_threshs.assign(values.begin(), values.end());
     }
     void set_sobelize_kernel_multipliers(int x, int y){
          this->sobel_kernel_multipliers.at(0) = x;
          this->sobel_kernel_multipliers.at(1) = y;
     }
     void set_contour_offset(int x, int y){
          this->contour_filtering_offset.x = x;
          this->contour_filtering_offset.y = y;
     }
};

class UmapProcessingDebugObjects{
public:
    UmapProcessingDebugObjects(){};

    cv::Mat preprocessing_keep_mask;
    std::vector< std::vector<cv::Point> > filtered_contours;
    std::vector< std::vector<cv::Point> > all_contours;
    std::vector< cv::Vec4i > filtered_hierarchies;
    std::vector< cv::Vec4i > all_hierarchies;
    void update(bool depth_based, float cam_baseline = 0, float cam_dscale = 0){}
};

#endif // VBOATS_UMAP_PROCESSING_PARAMS_H_
