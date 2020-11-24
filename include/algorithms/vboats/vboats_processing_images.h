#ifndef VBOATS_VBOATS_PROCESSING_IMAGES_H_
#define VBOATS_VBOATS_PROCESSING_IMAGES_H_

#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "obstacle.h"

class VboatsProcessingImages{
public:
     VboatsProcessingImages(){}

     // <custom-fold Misc
     bool visualize_umap_raw                 = false;
     bool visualize_vmap_raw                 = false;
     void enable_umap_raw_visualization(bool flag = true){ this->visualize_umap_raw = flag; }
     void enable_vmap_raw_visualization(bool flag = true){ this->visualize_vmap_raw = flag; }

     cv::Mat umap_raw;
     cv::Mat vmap_raw;
     void set_umap_raw(const cv::Mat& image){
          if(!this->visualize_umap_raw) this->umap_raw = cv::Mat();
          else this->umap_raw = image.clone();
     }
     void set_vmap_raw(const cv::Mat& image){
          if(!this->visualize_vmap_raw) this->vmap_raw = cv::Mat();
          else this->vmap_raw = image.clone();
     }
     // </custom-fold>

     // <custom-fold High-level debugging identifiers
     bool visualize_extracted_object_windows = false;
     void enable_extracted_object_windows_visualization(bool flag = true){ this->visualize_extracted_object_windows = flag; }

     int gnd_line_offset;
     std::vector<float> gnd_line_coefficients;
     std::vector< cv::Vec4i > filtered_umap_hierarchies;
     std::vector< std::vector<cv::Point> > filtered_umap_contours;
     std::vector<cv::Rect> found_obstacle_regions;
     std::vector< std::vector<cv::Rect> > vmap_object_search_regions;
     void set_filtered_umap_hierarchies(const std::vector< cv::Vec4i >& object){
          this->filtered_umap_hierarchies = std::vector<cv::Vec4i>{object.begin(), object.end()};
     }
     void set_filtered_umap_contours(const std::vector< std::vector<cv::Point> >& object){
          this->filtered_umap_contours = object;
     }
     void set_found_obstacle_regions(const std::vector<cv::Rect>& object){
          this->found_obstacle_regions = std::vector<cv::Rect>{object.begin(), object.end()};
     }
     void set_vmap_object_search_regions(const std::vector< std::vector<cv::Rect> >& object){
          this->vmap_object_search_regions = object;
     }
     void set_gnd_line_coefficients(std::vector<float> params){
          this->gnd_line_coefficients = std::vector<float>{params.begin(), params.end()};
     }
     void set_gnd_line_intercept_offset(int value){ this->gnd_line_offset = value; }
     // </custom-fold>

     // <custom-fold Mid-level debugging identifiers
     bool visualize_angle_corrected_depth    = false;
     bool visualize_umap_keep_mask           = false;
     bool visualize_vmap_post_keep_mask      = false;
     bool visualize_gnd_line_keep_mask       = false;
     bool visualize_obj_candidate_keep_mask  = false;
     bool visualize_vmap_candidates_img      = false;
     void enable_angle_corrected_depth_visualization(bool flag = true){ this->visualize_angle_corrected_depth = flag; }
     void enable_umap_keep_mask_visualization(bool flag = true){ this->visualize_umap_keep_mask = flag; }
     void enable_vmap_post_keep_mask_visualization(bool flag = true){ this->visualize_vmap_post_keep_mask = flag; }
     void enable_obj_candidate_keep_mask_visualization(bool flag = true){ this->visualize_obj_candidate_keep_mask = flag; }
     void enable_gnd_line_keep_mask_visualization(bool flag = true){ this->visualize_gnd_line_keep_mask = flag; }
     void enable_vmap_candidates_img_visualization(bool flag = true){ this->visualize_vmap_candidates_img = flag; }

     cv::Mat angle_corrected_depth_img;
     cv::Mat gnd_line_filtering_keep_mask;
     cv::Mat obj_candidate_filtering_keep_mask;
     cv::Mat umap_keep_mask;
     cv::Mat vmap_postproc_keep_mask;
     cv::Mat vmap_object_candidates_img;
     void set_angle_corrected_depth_image(const cv::Mat& image){
          if(!this->visualize_angle_corrected_depth) this->angle_corrected_depth_img = cv::Mat();
          else this->angle_corrected_depth_img = image.clone();
     }
     void set_gnd_line_filtering_keep_mask(const cv::Mat& image){
          if(!this->visualize_gnd_line_keep_mask) this->gnd_line_filtering_keep_mask = cv::Mat();
          else this->gnd_line_filtering_keep_mask = image.clone();
     }
     void set_obj_candidate_filtering_keep_mask(const cv::Mat& image){
          if(!this->visualize_obj_candidate_keep_mask) this->obj_candidate_filtering_keep_mask = cv::Mat();
          else this->obj_candidate_filtering_keep_mask = image.clone();
     }
     void set_umap_sobelized_keep_mask(const cv::Mat& image){
          if(!this->visualize_umap_keep_mask) this->umap_keep_mask = cv::Mat();
          else this->umap_keep_mask = image.clone();
     }
     void set_vmap_sobelized_postprocessed_keep_mask(const cv::Mat& image){
          if(!this->visualize_vmap_post_keep_mask) this->vmap_postproc_keep_mask = cv::Mat();
          else this->vmap_postproc_keep_mask = image.clone();
     }
     void set_vmap_object_candidates_image(const cv::Mat& image){
          if(!this->visualize_vmap_candidates_img) this->vmap_object_candidates_img = cv::Mat();
          else this->vmap_object_candidates_img = image.clone();
     }
     // </custom-fold>

     // <custom-fold U-Map low-level debugging
     bool visualize_umap_sobel_raw           = false;
     bool visualize_umap_sobel_preprocessed  = false;
     bool visualize_umap_sobel_dilated       = false;
     bool visualize_umap_sobel_blurred       = false;
     void enable_umap_sobel_raw_visualization(bool flag = true){ this->visualize_umap_sobel_raw = flag; }
     void enable_umap_sobel_preprocessed_visualization(bool flag = true){ this->visualize_umap_sobel_preprocessed = flag; }
     void enable_umap_sobel_dilated_visualization(bool flag = true){ this->visualize_umap_sobel_dilated = flag; }
     void enable_umap_sobel_blurred_visualization(bool flag = true){ this->visualize_umap_sobel_blurred = flag; }

     cv::Mat umap_sobel_raw;
     cv::Mat umap_sobel_preprocessed;
     cv::Mat umap_sobel_dilated;
     cv::Mat umap_sobel_blurred;
     void set_umap_sobelized_raw(const cv::Mat& image){
          if(!this->visualize_umap_sobel_raw) this->umap_sobel_raw = cv::Mat();
          else this->umap_sobel_raw = image.clone();
     }
     void set_umap_sobelized_pre_filtering(const cv::Mat& image){
          if(!this->visualize_umap_sobel_preprocessed) this->umap_sobel_preprocessed = cv::Mat();
          else this->umap_sobel_preprocessed = image.clone();
     }
     void set_umap_sobelized_dilated(const cv::Mat& image){
          if(!this->visualize_umap_sobel_dilated) this->umap_sobel_dilated = cv::Mat();
          else this->umap_sobel_dilated = image.clone();
     }
     void set_umap_sobelized_blurred(const cv::Mat& image){
          if(!this->visualize_umap_sobel_blurred) this->umap_sobel_blurred = cv::Mat();
          else this->umap_sobel_blurred = image.clone();
     }

     std::vector< cv::Vec4i > all_umap_hierarchies;
     std::vector< std::vector<cv::Point> > all_umap_contours;
     void set_all_umap_hierarchies(const std::vector< cv::Vec4i >& object){
          this->all_umap_hierarchies = std::vector<cv::Vec4i>{object.begin(), object.end()};
     }
     void set_all_umap_contours(const std::vector< std::vector<cv::Point> >& object){
          this->all_umap_contours = object;
     }
     // </custom-fold>

     // <custom-fold  V-Map low-level debugging
     bool visualize_vmap_post_sobel_threshed = false;
     bool visualize_vmap_post_sobel_blurred  = false;
     bool visualize_vmap_sobelized_preprocessed = false;
     void enable_vmap_post_sobel_threshed_visualization(bool flag = true){ this->visualize_vmap_post_sobel_threshed = flag; }
     void enable_vmap_post_sobel_blurred_visualization(bool flag = true){ this->visualize_vmap_post_sobel_blurred = flag; }
     void enable_vmap_sobelized_preprocessed_visualization(bool flag = true){ this->visualize_vmap_sobelized_preprocessed = flag; }

     cv::Mat vmap_postproc_sobel_threshed;
     cv::Mat vmap_postproc_sobel_blurred;
     cv::Mat vmap_sobelized_preprocessed;
     void set_vmap_sobelized_thresholded(const cv::Mat& image){
          if(!this->visualize_vmap_post_sobel_threshed) this->vmap_postproc_sobel_threshed = cv::Mat();
          else this->vmap_postproc_sobel_threshed = image.clone();
     }
     void set_vmap_sobelized_postprocessed_blurred(const cv::Mat& image){
          if(!this->visualize_vmap_post_sobel_blurred) this->vmap_postproc_sobel_blurred = cv::Mat();
          else this->vmap_postproc_sobel_blurred = image.clone();
     }
     void set_vmap_sobelized_preprocessed(const cv::Mat& image){
          if(!this->visualize_vmap_sobelized_preprocessed) this->vmap_sobelized_preprocessed = cv::Mat();
          else this->vmap_sobelized_preprocessed = image.clone();
     }
     //  </custom-fold>

     // Composite Image Construction Helpers
     cv::Mat overlay_ground_lines(const cv::Mat& image){
          cv::Mat display;
          if(image.empty()) return display;
          if(this->gnd_line_coefficients.empty()) return display;

          if(image.type() != CV_8UC3) display = imCvtCmap(image);
          else display = image.clone();

          int b = (int) this->gnd_line_coefficients[1];
          int yk = int(display.cols * this->gnd_line_coefficients[0]) + b;
          int yu = int(display.cols * this->gnd_line_coefficients[0]) + (b-this->gnd_line_offset);
          cv::line(display, cv::Point(0, b), cv::Point(display.cols, yk), cv::Scalar(0,255,0), 1, cv::LINE_AA);
          cv::line(display, cv::Point(0, (b-this->gnd_line_offset)), cv::Point(display.cols, yu), cv::Scalar(0,0,255), 1, cv::LINE_AA);
          return display;
     }
     cv::Mat overlay_objects_search_windows(const cv::Mat& image){
          cv::Mat display;
          if(image.empty()) return display;
          if( (this->vmap_object_search_regions.empty() )
               || (this->vmap_object_search_regions.size() <= 0)
          ) return display;

          if(image.type() != CV_8UC3) display = imCvtCmap(image);
          else display = image.clone();

          for(auto windows : this->vmap_object_search_regions){
               for(cv::Rect window : windows){ cv::rectangle(display, window, cv::Scalar(0, 255, 255), 1); }
          }
          return display;
     }
     cv::Mat overlay_umap_contours(const cv::Mat& image, bool show_all_contours = false){
          cv::Mat display;
          if(image.empty()) return display;
          if( (this->filtered_umap_contours.empty() )
               || (this->filtered_umap_contours.size() <= 0)
          ) return display;

          if(image.type() != CV_8UC3) display = imCvtCmap(image);
          else display = image.clone();

          if( (!this->all_umap_contours.empty() ) && (this->all_umap_contours.size() > 0)){
               for(int j = 0; j < int(this->all_umap_contours.size()); j++){
                    cv::drawContours(display, this->all_umap_contours, j, cv::Scalar(255,0,255), 1, cv::LINE_AA, this->all_umap_hierarchies, 0, cv::Point(0,0));
               }
          }

          for(int i = 0; i < int(this->filtered_umap_contours.size()); i++){
               cv::drawContours(display, this->filtered_umap_contours, i, cv::Scalar(0,255,0), 2, cv::LINE_AA, this->filtered_umap_hierarchies, 0, cv::Point(0,0));
          }

          return display;
     }
     cv::Mat overlay_obstacle_bounding_boxes(const cv::Mat& image, std::vector<Obstacle> obstacles){
          cv::Mat display;
          if(image.empty()) return display;
          if( (obstacles.empty()) || (obstacles.size() <= 0) ) return display;
          if(image.type() == CV_8UC3) display = image.clone();
          else{
               double minVal, maxVal;
               cv::minMaxLoc(image, &minVal, &maxVal);
               image.convertTo(display, CV_8UC1, (255.0/maxVal) );
               cv::cvtColor(display, display, cv::COLOR_GRAY2BGR);
          }
          // TODO: Hardcoded for now
          cv::Scalar box_color = cv::Scalar(0, 255, 255);
          for(Obstacle obj : obstacles){ cv::rectangle(display, obj.minXY, obj.maxXY, box_color, 3); }
          return display;
     }
     // TODO: Blend keep mask into original image
     // cv::Mat overlay_keep_mask(const cv::Mat& image, const cv::Mat& mask, double blend_alpha, cv::Scalar color){}

     /** Matplotlib method reference
     // Vmaps
     long nrows = 1;
     long ncols = (long) vmapsDisps.size();
     pplots(vmapsDisps, ncols, nrows, "Vmaps");

     // Umaps
     long ncols = 1;
     long nrows = (long) umapsDisps.size();
     pplots(umapsDisps, ncols, nrows, "Umaps", true);
     */
     cv::Mat construct_low_level_umap_image(bool set_titles = false){
          cv::Mat display;
          cv::Mat border_tile;
          int border_tile_gap = 5;

          // Begin adding tiles of valid or available images to the list for merging into one later
          cv::Mat tmpTileDisplay;
          std::vector< cv::Mat > img_tiles;
          if(this->visualize_umap_sobel_raw && (!this->umap_sobel_raw.empty()) ){
               // If the border tile size hasn't been determined yet set the size, otherwise skip this part
               if(border_tile.empty()){
                    cv::Mat border_tile(cv::Size(border_tile_gap, this->umap_sobel_raw.cols), CV_8UC3);
                    border_tile.setTo( cv::Scalar(255, 255, 255) );
               }
               tmpTileDisplay = imCvtCmap(this->umap_sobel_raw);
               if(!tmpTileDisplay.empty()){
                    if(set_titles) cv::putText(tmpTileDisplay, "Raw Sobel", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
                    img_tiles.push_back(tmpTileDisplay);
                    img_tiles.push_back(border_tile);
               }
          }
          if(this->visualize_umap_sobel_preprocessed && (!this->umap_sobel_preprocessed.empty()) ){
               // If the border tile size hasn't been determined yet set the size, otherwise skip this part
               if(border_tile.empty()){
                    cv::Mat border_tile(cv::Size(border_tile_gap, this->umap_sobel_preprocessed.cols), CV_8UC3);
                    border_tile.setTo( cv::Scalar(255, 255, 255) );
               }
               tmpTileDisplay = imCvtCmap(this->umap_sobel_preprocessed);
               if(!tmpTileDisplay.empty()){
                    if(set_titles) cv::putText(tmpTileDisplay, "Pre-Proc. Sobel", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
                    img_tiles.push_back(tmpTileDisplay);
                    img_tiles.push_back(border_tile);
               }
          }
          if(this->visualize_umap_sobel_dilated && (!this->umap_sobel_dilated.empty()) ){
               // If the border tile size hasn't been determined yet set the size, otherwise skip this part
               if(border_tile.empty()){
                    cv::Mat border_tile(cv::Size(border_tile_gap, this->umap_sobel_dilated.cols), CV_8UC3);
                    border_tile.setTo( cv::Scalar(255, 255, 255) );
               }
               tmpTileDisplay = imCvtCmap(this->umap_sobel_dilated);
               if(!tmpTileDisplay.empty()){
                    if(set_titles) cv::putText(tmpTileDisplay, "Dilated Sobel", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
                    img_tiles.push_back(tmpTileDisplay);
                    img_tiles.push_back(border_tile);
               }
          }
          if(this->visualize_umap_sobel_blurred && (!this->umap_sobel_blurred.empty()) ){
               // If the border tile size hasn't been determined yet set the size, otherwise skip this part
               if(border_tile.empty()){
                    cv::Mat border_tile(cv::Size(border_tile_gap, this->umap_sobel_blurred.cols), CV_8UC3);
                    border_tile.setTo( cv::Scalar(255, 255, 255) );
               }
               tmpTileDisplay = imCvtCmap(this->umap_sobel_blurred);
               if(!tmpTileDisplay.empty()){
                    if(set_titles) cv::putText(tmpTileDisplay, "Blurred Sobel", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
                    img_tiles.push_back(tmpTileDisplay);
                    img_tiles.push_back(border_tile);
               }
          }

          if(img_tiles.size() > 0){ cv::vconcat(img_tiles, display); }
          return display;
     }
     cv::Mat construct_low_level_vmap_image(bool set_titles = false){
          cv::Mat display;
          cv::Mat border_tile;
          int border_tile_gap = 5;
          // Begin adding tiles of valid or available images to the list for merging into one later
          cv::Mat tmpTileDisplay;
          std::vector< cv::Mat > img_tiles;
          if(this->visualize_vmap_sobelized_preprocessed && (!this->vmap_sobelized_preprocessed.empty()) ){
               // If the border tile size hasn't been determined yet set the size, otherwise skip this part
               if(border_tile.empty()){
                    cv::Mat border_tile(cv::Size(this->vmap_sobelized_preprocessed.rows, border_tile_gap), CV_8UC3);
                    border_tile.setTo( cv::Scalar(255, 255, 255) );
               }
               tmpTileDisplay = imCvtCmap(this->vmap_sobelized_preprocessed);
               if(!tmpTileDisplay.empty()){
                    if(set_titles) cv::putText(tmpTileDisplay, "Pre-Proc. Sobel", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
                    img_tiles.push_back(tmpTileDisplay);
                    img_tiles.push_back(border_tile);
               }
          }
          if(this->visualize_vmap_post_sobel_threshed && (!this->vmap_postproc_sobel_threshed.empty()) ){
               // If the border tile size hasn't been determined yet set the size, otherwise skip this part
               if(border_tile.empty()){
                    cv::Mat border_tile(cv::Size(this->vmap_postproc_sobel_threshed.rows, border_tile_gap), CV_8UC3);
                    border_tile.setTo( cv::Scalar(255, 255, 255) );
               }
               tmpTileDisplay = imCvtCmap(this->vmap_postproc_sobel_threshed);
               if(!tmpTileDisplay.empty()){
                    if(set_titles) cv::putText(tmpTileDisplay, "Post-Proc. Sobel Thresh\'d", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
                    img_tiles.push_back(tmpTileDisplay);
                    img_tiles.push_back(border_tile);
               }
          }
          if(this->visualize_vmap_post_sobel_blurred && (!this->vmap_postproc_sobel_blurred.empty()) ){
               // If the border tile size hasn't been determined yet set the size, otherwise skip this part
               if(border_tile.empty()){
                    cv::Mat border_tile(cv::Size(this->vmap_postproc_sobel_blurred.rows, border_tile_gap), CV_8UC3);
                    border_tile.setTo( cv::Scalar(255, 255, 255) );
               }
               tmpTileDisplay = imCvtCmap(this->vmap_postproc_sobel_blurred);
               if(!tmpTileDisplay.empty()){
                    if(set_titles) cv::putText(tmpTileDisplay, "Post-Proc. Sobel Blurred", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
                    img_tiles.push_back(tmpTileDisplay);
                    img_tiles.push_back(border_tile);
               }
          }

          if(img_tiles.size() > 0){ cv::hconcat(img_tiles, display); }
          return display;
     }

};

#endif // VBOATS_VBOATS_PROCESSING_IMAGES_H_
