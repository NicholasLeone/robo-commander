#include "algorithms/vboats/vboats.h"
#include "utilities/utils.h"
#include "algorithms/vboats/vboats_utils.h"

#include <math.h>                  // For fabs, ceil
#include <Eigen/Geometry>          // For Quaternion

using namespace std;

Vboats::Vboats(){}
Vboats::~Vboats(){}

void Vboats::init(){}
void Vboats::update(){}

cv::Mat Vboats::remove_umap_deadzones(const cv::Mat& umap){
     cv::Mat processed;
     if(umap.empty()) return processed;
     else processed = umap.clone();

     if(!this->_have_cam_info) return processed;
     int disparityFar = (int) ceil(this->_Tx / this->_hard_max_depth);

     // Create deadzone areas
     std::vector< std::vector<cv::Point> > deadzoneAreas;
     std::vector<cv::Point> deadzonePtsFar = {
          cv::Point(0, 0),
          cv::Point(umap.cols, 0),
          cv::Point(umap.cols, disparityFar),
          cv::Point(0, disparityFar),
          cv::Point(0, 0)
     };
     deadzoneAreas.push_back(deadzonePtsFar);

     // "blackout" deadzone regions
     cv::fillPoly(processed, deadzoneAreas, cv::Scalar(0));
     cv::boxFilter(processed, processed, -1, cv::Size(2,2));
     return processed;
}
cv::Mat Vboats::remove_vmap_deadzones(const cv::Mat& vmap){
     cv::Mat processed;
     if(vmap.empty()) return processed;
     else processed = vmap.clone();

     if(!this->_have_cam_info) return processed;
     int disparityFar = (int) ceil(this->_Tx / this->_hard_max_depth);
     int disparityNear = (int) ceil(this->_Tx / this->_hard_min_depth);

     vector<vector<cv::Point>> deadzoneAreas;
     vector<cv::Point> deadzonePtsFar = {
          cv::Point(0, 0),
          cv::Point(0, vmap.rows),
          cv::Point(disparityFar, vmap.rows),
          cv::Point(disparityFar, 0),
          cv::Point(0, 0)
     };
     deadzoneAreas.push_back(deadzonePtsFar);

     vector<cv::Point> deadzonePtsNear = {
          cv::Point(vmap.cols, 0),
          cv::Point(vmap.cols, vmap.rows),
          cv::Point(disparityNear, vmap.rows),
          cv::Point(disparityNear, 0),
          cv::Point(vmap.cols, 0)
     };
     deadzoneAreas.push_back(deadzonePtsNear);

     // "blackout" deadzone regions
     cv::fillPoly(processed, deadzoneAreas, cv::Scalar(0));
     return processed;
}
cv::Mat Vboats::generate_disparity_from_depth(const cv::Mat& depth){
     cv::Mat output;
     if(depth.empty()) return output;
     if(!this->_have_cam_info) return output;

     cv::Mat image;
     float gain = this->_depth2disparityFactor;
     if(depth.type() == CV_16UC1){
          depth.convertTo(image, CV_32F, this->_depth_scale);
          gain = gain * this->_depth_scale;
     } else if(depth.type() != CV_32F) depth.convertTo(image, CV_32F);
     else image = depth.clone();

     ForEachDepthConverter<float> d2dconverter(gain, (float) this->_hard_min_depth, (float) this->_hard_max_depth);
     image.forEach<float>(d2dconverter);

     if(image.type() != CV_8UC1){
          double minVal, maxVal;
          cv::minMaxLoc(image, &minVal, &maxVal);
          image.convertTo(output, CV_8UC1, (255.0/maxVal) );
     } else output = image.clone();
     return output.clone();
}

void Vboats::set_camera_info(cv::Mat K, float depth_scale, float baseline, bool verbose){
     if(this->_cam_info_count <= 10){
          this->_fx = (float)K.at<double>(0);
          this->_fy = (float)K.at<double>(4);
          this->_px = (float)K.at<double>(2);
          this->_py = (float)K.at<double>(5);
          this->_baseline = baseline;
          this->_depth_scale = depth_scale;
          this->_Tx = (float)(this->_fx * baseline);
          this->_depth2disparityFactor = this->_Tx / this->_depth_scale;

          if(verbose) printf("[INFO] %s::set_camera_info() ---- DepthScale = %.4f, Baseline = %.6f,  Focals = [%.2f, %.2f],  Principles = [%.2f, %.2f]\r\n", this->classLbl.c_str(), this->_depth_scale, this->_baseline, this->_fx, this->_fy, this->_px, this->_py);
          this->_have_cam_info = true;
     }
     this->_cam_info_count++;
}
void Vboats::set_camera_orientation(double roll, double pitch, double yaw, bool verbose){
     this->_cam_roll  = roll;
     this->_cam_pitch = pitch;
     this->_cam_yaw   = yaw;

     if(verbose){
          printf("[INFO] %s::set_camera_orientation() --- "
               "Camera RPY = [%.4lf, %.4lf, %.4lf] [rad]\t\t (%.2lf, %.2lf, %.2lf) (deg)\r\n", this->classLbl.c_str(),
               roll, pitch, yaw, roll*M_RAD2DEG, pitch*M_RAD2DEG, yaw*M_RAD2DEG
          );
     }
}
void Vboats::set_camera_orientation(double x, double y, double z, double w, bool verbose){
     Eigen::Quaternion<double> quat(w, x, y, z);
     Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);

     double roll  = (double) euler[0];
     double pitch = (double) euler[1];
     double yaw   = (double) euler[2];

     this->_cam_roll  = roll;
     this->_cam_pitch = pitch;
     this->_cam_yaw   = yaw;

     if(verbose){
          printf("[INFO] %s::set_camera_orientation() --- "
               "Camera RPY = [%.4lf, %.4lf, %.4lf] [rad]\t\t (%.2lf, %.2lf, %.2lf) (deg)\r\n", this->classLbl.c_str(),
               roll, pitch, yaw, roll*M_RAD2DEG, pitch*M_RAD2DEG, yaw*M_RAD2DEG
          );
     }
}
void Vboats::set_camera_angle_offset(double offset_degrees){ this->_cam_angle_offset = offset_degrees * M_DEG2RAD; }
void Vboats::set_depth_denoising_kernel_size(int size){ this->_filtered_depth_denoising_size = size; }

int Vboats::process(const cv::Mat& depth){
     if(depth.empty()){
          printf("[WARNING] %s::process() --- Depth input is empty.\r\n", this->classLbl.c_str());
          return -1;
     }
     if(!this->_have_cam_info){
          printf("[WARNING] %s::process() --- Camera Information Unknown.\r\n", this->classLbl.c_str());
          return -2;
     }

     cv::Mat depthRaw = depth.clone();
     cv::Mat disparity = this->generate_disparity_from_depth(depthRaw);
     if(disparity.empty()){
          printf("[WARNING] %s::process() --- Disparity input is empty.\r\n", this->classLbl.c_str());
          return -3;
     }
     cv::Mat disparityRaw = disparity.clone();

     cv::Mat depthInput, disparityInput;
     double correctionAngle = -this->_cam_roll - this->_cam_angle_offset;
     if(this->_do_angle_correction){
          cv::Mat warpedDepth = rotate_image(depthRaw, correctionAngle);
          cv::Mat warpeddDisparity = rotate_image(disparityRaw, correctionAngle);

          if(!warpedDepth.empty() && !warpeddDisparity.empty()){
               depthInput = warpedDepth.clone();
               disparityInput = warpeddDisparity.clone();
               this->_angle_correction_performed = true;
          } else{
               depthInput = depthRaw;
               disparityInput = disparityRaw;
               this->_angle_correction_performed = false;
          }
     } else{
          depthInput = depthRaw;
          disparityInput = disparityRaw;
          this->_angle_correction_performed = false;
     }

     cv::Mat umap, vmap;
     genUVMapThreaded(disparityInput, &umap, &vmap, 2.0);

     if(umap.empty()){
          printf("[WARNING] %s::process() --- Umap input is empty.\r\n", this->classLbl.c_str());
          return -4;
     }
     if(vmap.empty()){
          printf("[WARNING] %s::process() --- Vmap input is empty.\r\n", this->classLbl.c_str());
          return -5;
     }
     cv::Mat umapRaw = umap.clone();
     cv::Mat vmapRaw = vmap.clone();

     // cv::Mat umapDisplay = umap.clone();
     // cv::Mat vmapDisplay = vmap.clone();
     // cv::Mat depthDisplay = depth.clone();
     // cv::Mat disparityDisplay = disparity.clone();

     // Pre-process Umap
     cv::Mat uProcessed = this->remove_umap_deadzones(umapRaw);
     if(this->_umapFiltMeth == SOBELIZED_METHOD){
          uProcessed = preprocess_umap_sobelized(uProcessed,
               this->umapParams.sobel_thresh_pre_sobel,
               this->umapParams.sobel_thresh_sobel_preprocess,
               this->umapParams.sobel_thresh_sobel_postprocess,
               this->umapParams.sobel_dilate_size, this->umapParams.sobel_blur_size,
               this->umapParams.sobel_kernel_multipliers,
               nullptr,  // keep_mask visualization
               nullptr,  // sobel_raw visualization
               nullptr,  // sobel_preprocessed visualization
               nullptr,  // sobel_dilated visualization
               nullptr   // sobel_blurred visualization
          );
     } else if(this->_umapFiltMeth == STRIPPING_METHOD) uProcessed = preprocess_umap_stripping(uProcessed, &this->umapParams.stripping_threshs);

     // Find contours in Umap needed later for obstacle filtering
     vector<vector<cv::Point>> filtered_contours;
     find_contours(uProcessed, &filtered_contours, (int) this->_contourFiltMeth,
          this->umapParams.contour_filtering_thresh_min,
          this->umapParams.contour_filtering_thresh_max,
          &this->umapParams.contour_filtering_offset,
          &this->_umap_debugger.filtered_hierarchies,  // filtered contours visualization
          nullptr,  // all_contours visualization
          nullptr   // all_hierarchies visualization
     );

     // Pre-process Vmap
     cv::Mat preprocessedVmap = this->remove_vmap_deadzones(vmapRaw);
     if(this->_vmapFiltMeth == SOBELIZED_METHOD){
          preprocessedVmap = preprocess_vmap_sobelized(preprocessedVmap,
               this->vmapParams.sobel_preprocessing_thresh_sobel,
               this->vmapParams.sobel_preprocessing_blur_size,
               this->vmapParams.sobel_kernel_multipliers
          );
     } else if(this->_vmapFiltMeth == STRIPPING_METHOD) preprocessedVmap = preprocess_vmap_stripping(preprocessedVmap, &this->vmapParams.stripping_threshs);

     // Extract ground line parameters (if ground is present)
     std::vector<float> line_params;
     bool gndPresent = find_ground_line(preprocessedVmap, &line_params,
          this->vmapParams.gnd_line_search_min_deg,
          this->vmapParams.gnd_line_search_max_deg,
          this->vmapParams.gnd_line_search_deadzone,
          this->vmapParams.gnd_line_search_hough_thresh
     );
     if(!gndPresent) printf("[INFO] %s::process() --- Unable to detect Ground Line.\r\n", this->classLbl.c_str());

     // Finish V-map processing starting from the pre-processed vmap
     cv::Mat vProcessed = preprocessedVmap.clone();
     if(this->_vmapFiltMeth == SOBELIZED_METHOD){
          vProcessed = postprocess_vmap_sobelized(vmapRaw, preprocessedVmap,
               this->vmapParams.sobel_postprocessing_thresh_prefiltering,
               this->vmapParams.sobel_postprocessing_thresh_postfiltering,
               this->vmapParams.sobel_postprocessing_blur_size,
               this->vmapParams.sobel_kernel_multipliers,
               nullptr,  // sobel_threshed visualization
               nullptr,  // sobel_blurred visualization
               nullptr   // keep_mask visualization
          );
     }

     // Obstacle data extraction
     int nObs = 0;
     std::vector<Obstacle> obstacles_;
     std::vector< std::vector<cv::Rect> > obstacleRegions;
     if(this->_do_obstacle_data_extraction){
          nObs = find_obstacles_disparity(vProcessed, filtered_contours, line_params, &obstacles_,
               &obstacleRegions    // obstacle regions visualization
          );
     }

     // Filter the original depth image using all the useful data encoded within
     // the resulting processed UV-Maps
     cv::Mat filtered_image;
     int err = filter_depth_using_object_candidate_regions(depthInput, disparityInput,
          vProcessed, filtered_contours, &filtered_image, line_params,
          this->vmapParams.depth_filtering_gnd_line_intercept_offset,
          nullptr,  // keep_mask visualization
          nullptr,  // vmap_objects visualization
          nullptr   // vmap_search_regions visualization
     );

     cv::Mat filtered_depth;
     err = filter_depth_using_ground_line(filtered_image, disparityInput,
          vProcessed, line_params, &filtered_depth,
          std::vector<int>{this->vmapParams.depth_filtering_gnd_line_intercept_offset},
          nullptr   // keep_mask visualization
     );

     // Attempt to remove any noise (speckles) from the resulting filtered depth image
     cv::Mat final_depth;
     if(this->_denoise_filtered_depth){
          cv::Mat morphElement = cv::getStructuringElement(cv::MORPH_ELLIPSE,
               cv::Size(this->_filtered_depth_denoising_size, this->_filtered_depth_denoising_size)
          );
          cv::Mat morphedDepth;
          cv::morphologyEx(filtered_depth, morphedDepth, cv::MORPH_OPEN, morphElement);
          final_depth = morphedDepth.clone();
     } else final_depth = filtered_depth.clone();

     if(this->_do_angle_correction && this->_angle_correction_performed) final_depth = rotate_image(final_depth, -correctionAngle);

     // Return Output images if requested before visualization
     // if(obstacles) *obstacles = obstacles_;
     // if(filtered) *filtered = filtered_depth.clone();
     // if(processed_umap) *processed_umap = uProcessed.clone();
     // if(processed_vmap) *processed_vmap = vProcessed.clone();

     return (int) obstacles_.size();
}

/**
     if(this->_do_cv_wait_key){
          // if(this->_visualize_gnd_mask) imshowCmap(gndMask, "Ground-Line Mask");
          // if(this->_visualize_obj_mask) imshowCmap(objMask, "Object Mask");
          // if(this->_visualize_gnd_filter_img) imshowCmap(noGndImg, "Ground-Line Filtered Image");
          // if(this->_visualize_generated_disparity) imshowCmap(disparityCopy, "Proc. Input Disparity");
          if(this->_do_misc_viz){
               cv::Mat tmpDepthDisp;
               std::vector<cv::Mat> depthDisps;
               if(this->_visualize_procinput_depth && (!depthCopy.empty()) ){
                    tmpDepthDisp = imCvtCmap(depthCopy);
                    cv::putText(tmpDepthDisp, "Raw Depth", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
                    depthDisps.push_back(tmpDepthDisp);
               }
               if(this->_visualize_filtered_depth && (!filtered_depth.empty()) ){
                    tmpDepthDisp = imCvtCmap(filtered_depth);
                    cv::putText(tmpDepthDisp, "Filtered Depth", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
                    depthDisps.push_back(tmpDepthDisp);
               }
               cv::Mat depthsMerged;
               if(depthDisps.size() > 0){
                    long nrows = 1;
                    long ncols = (long) depthDisps.size();
                    if(this->_viz_pause_on_misc) pplots(depthDisps, ncols, nrows, "Depths", true);
                    else{
                         pplots(depthDisps, ncols, nrows, "Depths");
                         // if(this->_viz_sleep_secs <= 0) plt::pause(0.001);
                         // else plt::pause(this->_viz_sleep_secs);
                    }
                    // cv::hconcat(depthDisps, depthsMerged);
                    // if(!depthsMerged.empty()){
                    //      cv::namedWindow("Depths", cv::WINDOW_NORMAL);
                    //      cv::imshow("Depths", depthsMerged);
                    // }
               }
          }

          if(this->_do_vmap_viz){
               std::vector<cv::Mat> vmapsDisps;
               if(this->_visualize_vmap_raw || this->_visualize_vmap_raw_w_lines){
                    cv::Mat vmapCmap = imCvtCmap(vmapCopy);
                    if(this->_visualize_vmap_raw && (!vmapCmap.empty()) ){
                         cv::putText(vmapCmap, "Proc. Input Vmap", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
                         vmapsDisps.push_back(vmapCmap);
                    }
                    // if(this->_visualize_vmap_raw_w_lines && (!vmapCmap.empty()) && gndPresent ){
                    //      cv::Mat lineDisplay = vmapCopy.clone();
                    //      cv::cvtColor(lineDisplay, lineDisplay, cv::COLOR_GRAY2BGR);
                    //      int yk = int(vmapCmap.cols * gndM) + gndB;
                    //      int yu = int(vmapCmap.cols * gndM) + (gndB-this->_gnd_line_upper_offset);
                    //      int yl = int(vmapCmap.cols * gndM) + (gndB+this->_gnd_line_lower_offset);
                    //      cv::line(lineDisplay, cv::Point(0, gndB), cv::Point(vmapCmap.cols, yk), cv::Scalar(0,255,0), 2, cv::LINE_AA);
                    //      cv::line(lineDisplay, cv::Point(0, (gndB-this->_gnd_line_upper_offset)), cv::Point(vmapCmap.cols, yu), cv::Scalar(255,0,0), 2, cv::LINE_AA);
                    //      cv::line(lineDisplay, cv::Point(0, (gndB+this->_gnd_line_lower_offset)), cv::Point(vmapCmap.cols, yl), cv::Scalar(0,0,255), 2, cv::LINE_AA);
                    //      cv::putText(lineDisplay, "Ground Lines", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
                    //      vmapsDisps.push_back(lineDisplay);
                    // }
               }
               cv::Mat tmpVmapDisp;

               // if(this->_visualize_vmap_blurred && (!blurSobel.empty())){
               //      tmpVmapDisp = imCvtCmap(blurSobel);
               //      if(!tmpVmapDisp.empty()){
               //           cv::putText(tmpVmapDisp, "Vmap Blurred", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
               //           vmapsDisps.push_back(tmpVmapDisp);
               //      }
               // }
               // if(this->_visualize_vmap_dilated && (!dilatedSobel.empty())){
               //      tmpVmapDisp = imCvtCmap(dilatedSobel);
               //      if(!tmpVmapDisp.empty()){
               //           cv::putText(tmpVmapDisp, "Vmap Dilated", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
               //           vmapsDisps.push_back(tmpVmapDisp);
               //      }
               // }

               if(this->_visualize_vmap_sobel_raw && (!preprocessedVmap.empty()) ){
                    tmpVmapDisp = imCvtCmap(preprocessedVmap);
                    if(!tmpVmapDisp.empty()){
                         cv::putText(tmpVmapDisp, "Pre-Proc. Sobel", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
                         vmapsDisps.push_back(tmpVmapDisp);
                    }
               }

               // if(this->_visualize_vmap_thresh && (!sobelThresh.empty()) ){
               //      tmpVmapDisp = imCvtCmap(sobelThresh);
               //      if(!tmpVmapDisp.empty()){
               //           cv::putText(tmpVmapDisp, "Vmap Thresholded", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
               //           vmapsDisps.push_back(tmpVmapDisp);
               //      }
               // }
               // if(this->_visualize_vmap_sec_thresh && (!sobelSecThresh.empty()) ){
               //      tmpVmapDisp = imCvtCmap(sobelSecThresh);
               //      if(!tmpVmapDisp.empty()){
               //           cv::putText(tmpVmapDisp, "Sobel Secondary Thresholding", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
               //           vmapsDisps.push_back(tmpVmapDisp);
               //      }
               // }
               // if(this->_visualize_vmap_sec_dilated && (!sobelSecDilate.empty()) ){
               //      tmpVmapDisp = imCvtCmap(sobelSecDilate);
               //      if(!tmpVmapDisp.empty()){
               //           cv::putText(tmpVmapDisp, "Sobel Secondary Dilate", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
               //           vmapsDisps.push_back(tmpVmapDisp);
               //      }
               // }
               // if(this->_visualize_vmap_sec_blur && (!sobelSecBlur.empty()) ){
               //      tmpVmapDisp = imCvtCmap(sobelSecBlur);
               //      if(!tmpVmapDisp.empty()){
               //           cv::putText(tmpVmapDisp, "Sobel Secondary Blurring", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
               //           vmapsDisps.push_back(tmpVmapDisp);
               //      }
               // }
               // if(this->_visualize_vmap_mask && (!segMask.empty()) ){
               //      tmpVmapDisp = imCvtCmap(segMask);
               //      if(!tmpVmapDisp.empty()){
               //           cv::putText(tmpVmapDisp, "Vmap Mask", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
               //           vmapsDisps.push_back(tmpVmapDisp);
               //      }
               // }

               if(this->_visualize_process_input_vmap || this->_visualize_vmap_sobel_filtered || this->_visualize_obstacle_windows){
                    cv::Mat vmapProcDisp = imCvtCmap(vProcessed);
                    if( (this->_visualize_process_input_vmap || this->_visualize_vmap_sobel_filtered) && (!vmapProcDisp.empty()) ){
                         cv::Mat tmpDisp = vmapProcDisp.clone();
                         cv::putText(tmpDisp, "Post-Proc. Sobel", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
                         vmapsDisps.push_back(tmpDisp);
                    }
                    if(this->_visualize_obstacle_windows && (!vmapProcDisp.empty()) && (objectsWindows.size() != 0) ){
                         cv::Mat windowDisplay = vmapProcDisp.clone();
                         for(auto objWindows : objectsWindows){
                              for(cv::Rect window : objWindows){
                                   cv::rectangle(windowDisplay, window, cv::Scalar(0, 255, 255), 1);
                              }
                         }
                         if(this->_visualize_vmap_raw_w_lines && (!vProcessed.empty()) && gndPresent ){
                              int yk = int(windowDisplay.cols * gndM) + gndB;
                              int yu = int(windowDisplay.cols * gndM) + (gndB-this->_gnd_line_upper_offset);
                              int yl = int(windowDisplay.cols * gndM) + (gndB+this->_gnd_line_lower_offset);
                              cv::line(windowDisplay, cv::Point(0, gndB), cv::Point(windowDisplay.cols, yk), cv::Scalar(0,255,0), 1, cv::LINE_AA);
                              cv::line(windowDisplay, cv::Point(0, (gndB-this->_gnd_line_upper_offset)), cv::Point(windowDisplay.cols, yu), cv::Scalar(0,0,255), 1, cv::LINE_AA);
                              cv::line(windowDisplay, cv::Point(0, (gndB+this->_gnd_line_lower_offset)), cv::Point(windowDisplay.cols, yl), cv::Scalar(255,0,255), 1, cv::LINE_AA);
                         }
                         vmapsDisps.push_back(windowDisplay);
                    }
               }

               // if(this->_visualize_obj_detection_vmap){
               //      tmpVmapDisp = imCvtCmap(obsSearchVmap);
               //      if(!tmpVmapDisp.empty()){
               //           cv::putText(tmpVmapDisp, "Object Detection Vmap", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
               //           vmapsDisps.push_back(tmpVmapDisp);
               //      }
               // }

               cv::Mat vmapsMerged;
               if(vmapsDisps.size() > 0){
                    long nrows = 1;
                    long ncols = (long) vmapsDisps.size();
                    if(this->_viz_pause_on_vmap) pplots(vmapsDisps, ncols, nrows, "Vmaps", true);
                    else{
                         pplots(vmapsDisps, ncols, nrows, "Vmaps");
                         // if(this->_viz_sleep_secs <= 0) plt::pause(0.001);
                         // else plt::pause(this->_viz_sleep_secs);
                    }

                    // plt::pause(0.01);
                    // cv::hconcat(vmapsDisps, vmapsMerged);
                    // if(!vmapsMerged.empty()){
                    //      cv::namedWindow("Vmaps", cv::WINDOW_NORMAL);
                    //      cv::imshow("Vmaps", vmapsMerged);
                    // }
               }
          }
          if(this->_do_umap_viz){
               std::vector<cv::Mat> umapsDisps;
               cv::Mat tmpUmapDisp;
               if(this->_visualize_umap_raw && (!umapCopy.empty()) ){
                    tmpUmapDisp = imCvtCmap(umapCopy);
                    if(this->_visualize_umap_contours && (contours.size() > 0) ){
                         for(int i = 0; i < int(contours.size()); i++){ cv::drawContours(tmpUmapDisp, contours, i, cv::Scalar(0,255,0), 1, cv::LINE_AA, hierarchies, 0, cv::Point(0,0)); }
                    }
                    cv::putText(tmpUmapDisp, "Raw Umap", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
                    umapsDisps.push_back(tmpUmapDisp);
               }
               if(this->_visualize_umap_filtered && (!uProcessed.empty()) ){
                    tmpUmapDisp = imCvtCmap(uProcessed);
                    if(this->_visualize_umap_contours && (contours.size() > 0) ){
                         for(int i = 0; i < int(contours.size()); i++){ cv::drawContours(tmpUmapDisp, contours, i, cv::Scalar(0,255,0), 1, cv::LINE_AA, hierarchies, 0, cv::Point(0,0)); }
                    }
                    cv::putText(tmpUmapDisp, "Filtered Umap", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
                    umapsDisps.push_back(tmpUmapDisp);
               }
               cv::Mat umapsMerged;
               if(umapsDisps.size() > 0){
                    long ncols = 1; long nrows = (long) umapsDisps.size();
                    if(this->_viz_pause_on_umap) pplots(umapsDisps, ncols, nrows, "Umaps", true);
                    else{ pplots(umapsDisps, ncols, nrows, "Umaps");
                         // if(this->_viz_sleep_secs <= 0) plt::pause(0.001);
                         // else plt::pause(this->_viz_sleep_secs);
                    }

                    // plt::pause(0.01);
                    // cv::vconcat(umapsDisps, umapsMerged);
                    // if(!umapsMerged.empty()){
                    //      cv::namedWindow("Umaps", cv::WINDOW_NORMAL);
                    //      cv::imshow("Umaps", umapsMerged);
                    // }
               }
          }
     }
     if(this->_do_cv_wait_key){
          // cv::waitKey(this->_viz_sleep_ms);
          if(this->_viz_sleep_secs <= 0) plt::pause(0.001);
          else plt::pause(this->_viz_sleep_secs);
     }

     return (int) _obstacles.size();
}
*/

void Vboats::set_absolute_minimum_depth(float value){ this->_hard_min_depth = value; }
void Vboats::set_absolute_maximum_depth(float value){ this->_hard_max_depth = value; }
void Vboats::set_contour_filtering_method(std::string method){
     if(strcmp(method.c_str(), "perimeter") == 0) this->_contourFiltMeth = PERIMETER_BASED;
     else if(strcmp(method.c_str(), "Perimeter") == 0) this->_contourFiltMeth = PERIMETER_BASED;
     else if(strcmp(method.c_str(), "area") == 0) this->_contourFiltMeth = AREA_BASED;
     else if(strcmp(method.c_str(), "Area") == 0) this->_contourFiltMeth = AREA_BASED;
     else this->_contourFiltMeth = PERIMETER_BASED;
}
void Vboats::set_umap_processing_method(std::string method){
     if(strcmp(method.c_str(), "strips") == 0) this->_umapFiltMeth = STRIPPING_METHOD;
     else if(strcmp(method.c_str(), "stripping") == 0) this->_umapFiltMeth = STRIPPING_METHOD;
     else if(strcmp(method.c_str(), "sobel") == 0) this->_umapFiltMeth = SOBELIZED_METHOD;
     else if(strcmp(method.c_str(), "sobelized") == 0) this->_umapFiltMeth = SOBELIZED_METHOD;
     else this->_umapFiltMeth = SOBELIZED_METHOD;
}
void Vboats::set_vmap_processing_method(std::string method){
     if(strcmp(method.c_str(), "strips") == 0) this->_vmapFiltMeth = STRIPPING_METHOD;
     else if(strcmp(method.c_str(), "stripping") == 0) this->_vmapFiltMeth = STRIPPING_METHOD;
     else if(strcmp(method.c_str(), "sobel") == 0) this->_vmapFiltMeth = SOBELIZED_METHOD;
     else if(strcmp(method.c_str(), "sobelized") == 0) this->_vmapFiltMeth = SOBELIZED_METHOD;
     else this->_vmapFiltMeth = SOBELIZED_METHOD;
}
void Vboats::set_image_angle_correction_type(std::string method){
     if(strcmp(method.c_str(), "roll") == 0) this->_angleCorrectionType = ROLL_CORRECTION;
     else if(strcmp(method.c_str(), "Roll") == 0) this->_angleCorrectionType = ROLL_CORRECTION;
     else if(strcmp(method.c_str(), "pitch") == 0) this->_angleCorrectionType = PITCH_CORRECTION;
     else if(strcmp(method.c_str(), "Pitch") == 0) this->_angleCorrectionType = PITCH_CORRECTION;
     else if(strcmp(method.c_str(), "yaw") == 0) this->_angleCorrectionType = YAW_CORRECTION;
     else if(strcmp(method.c_str(), "Yaw") == 0) this->_angleCorrectionType = YAW_CORRECTION;
     else this->_angleCorrectionType = ROLL_CORRECTION;
}

void Vboats::enable_angle_correction(bool flag){ this->_do_angle_correction = flag; }
void Vboats::enable_filtered_depth_denoising(bool flag){ this->_denoise_filtered_depth = flag; }
void Vboats::enable_obstacle_data_extraction(bool flag){ this->_do_obstacle_data_extraction = flag; }
