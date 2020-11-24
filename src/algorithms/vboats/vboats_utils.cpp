#include "base/definitions.h"
#include "utilities/utils.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "utilities/image_utils.h"
#include "algorithms/vboats/vboats_utils.h"

using namespace std;

#define VERBOSE_OBS_SEARCH false

/** =========================
*     UV-Map Generation
* =========================== */
void genUVMap(cv::Mat image, cv::Mat* umap, cv::Mat* vmap, bool verbose){
     if(image.empty()){
          if(verbose) printf("[WARN] genUVMap() --- Input image is empty, not generating UV-maps.\r\n");
          return;
     }
     double minVal, maxVal;
     cv::minMaxLoc(image, &minVal, &maxVal);
     int dmax = (int) maxVal + 1;
     if(verbose) printf("dmax = %d, min, max = %.2f, %.2f, h = %d, w = %d\r\n",dmax,minVal, maxVal,image.rows,image.cols);

     cv::Mat umapMat = cv::Mat::zeros(dmax, image.cols, CV_8UC1);
     /** Size Temporarily transposed for ease of for-loop copying (Target size = h x dmax)*/
     cv::Mat vmapMat = cv::Mat::zeros(dmax, image.rows, CV_8UC1);

     int channels[] = {0};
     int histSize[] = {dmax};
     float sranges[] = { 0, dmax };
     const float* ranges[] = { sranges };

     /** Non-parallized Umap */
     cv::MatND histU;
	for(int i = 0; i < image.cols; i++){
		cv::Mat uscan = image.col(i);
		cv::calcHist(&uscan, 1, channels, cv::Mat(), histU, 1, histSize, ranges);
		histU.col(0).copyTo(umapMat.col(i));
	}
     /** Non-parallized Vmap */
     cv::MatND histV;
	for(int j = 0; j < image.rows; j++){
          cv::Mat vscan = image.row(j);
          cv::calcHist(&vscan, 1, channels, cv::Mat(), histV, 1, histSize, ranges);
          histV.col(0).copyTo(vmapMat.col(j));
	}
     /** Correct generated vmap's dimensions */
     vmapMat = vmapMat.t();
     if(umap) *umap = umapMat;
     if(vmap) *vmap = vmapMat;
}
void genUVMapThreaded(cv::Mat image, cv::Mat* umap, cv::Mat* vmap, double nThreads, bool verbose){
     if(image.empty()){
          if(verbose) printf("[WARN] genUVMapThreaded() --- Input image is empty, not generating UV-maps.\r\n");
          return;
     }
     double minVal, maxVal;
     cv::minMaxLoc(image, &minVal, &maxVal);
     int dmax = (int) maxVal + 1;

     cv::Mat umapMat = cv::Mat::zeros(dmax, image.cols, CV_8UC1);
     /** Size Temporarily transposed for ease of for-loop copying (Target size = h x dmax)*/
     cv::Mat vmapMat = cv::Mat::zeros(dmax, image.rows, CV_8UC1);

     int channels[] = {0};
     int histSize[] = {dmax};
     float sranges[] = { 0, dmax };
     const float* ranges[] = { sranges };

     /** Parallized Umap */
     cv::parallel_for_(cv::Range(0,image.cols), [&](const cv::Range& range){
          cv::MatND histU;
          for(int i = range.start; i < range.end; i++){
               cv::Mat uscan = image.col(i);
               cv::calcHist(&uscan, 1, channels, cv::Mat(), histU, 1, histSize, ranges);
               histU.col(0).copyTo(umapMat.col(i));
          }
     },nThreads);

     /** Parallized Vmap */
     cv::parallel_for_(cv::Range(0, image.rows), [&](const cv::Range& range){
          cv::MatND histV;
          for(int j = range.start; j < range.end; j++){
               cv::Mat vscan = image.row(j);
               cv::calcHist(&vscan, 1, channels, cv::Mat(), histV, 1, histSize, ranges);
               histV.col(0).copyTo(vmapMat.col(j));
          }
     },nThreads);
     /** Correct generated vmap's dimensions */
     vmapMat = vmapMat.t();
     if(umap) *umap = umapMat;
     if(vmap) *vmap = vmapMat;
}

/** =========================
*      UV-Map Processing
* =========================== */
cv::Mat preprocess_umap_stripping(const cv::Mat& input, vector<float>* thresholds, bool verbose, bool debug){
     cv::Mat output;
     preprocess_umap_stripping(input, &output, thresholds, verbose, debug);
     return output;
}
void preprocess_umap_stripping(const cv::Mat& input, cv::Mat* output,
     vector<float>* thresholds, bool verbose, bool debug)
{
     cv::Mat filtered;
     if(input.empty()) return;
     cv::Mat imgCopy = input.clone();

     vector<float> threshs;
     if(thresholds) threshs = (*thresholds);
     if(threshs.empty()) threshs = vector<float>{0.3,0.295,0.275,0.3};
     int nThreshs = threshs.size();

     if(verbose){
          std::string threshStr = vector_str(threshs, ", ");
          printf("[INFO] preprocess_umap_stripping() --- Using %d thresholds = [%s]\r\n", (int)threshs.size(), threshStr.c_str());
     }

     double uMin, uMax;
     cv::minMaxLoc(imgCopy, &uMin, &uMax);

     float ratio = (float)(imgCopy.rows) / 255.0;
     int nStrips = int(ceil(ratio * nThreshs)) - 1;
     if(debug) printf("uMax, sizeRatio, nThreshs: %d, %.2f, %d\r\n", (int) uMax, ratio, nStrips);

     cv::threshold(imgCopy, imgCopy, 8, 255, cv::THRESH_TOZERO);
     std::vector<cv::Mat> strips;
     int err = strip_image(imgCopy, &strips, nStrips, false);

     int idx = 0;
     std::vector<cv::Mat> pStrips;
     if(err >= 0){
          for(cv::Mat strip : strips){
               double stripMin, stripMax;
               cv::Scalar stripMean, stripStddev;
               cv::minMaxLoc(strip, &stripMin, &stripMax);
               cv::meanStdDev(strip, stripMean, stripStddev);
               int tmpMax = (int) stripMax;
               double tmpMean = (double) stripMean[0];
               double tmpStd = (double) stripStddev[0];
               if(tmpMean == 0.0){
                    pStrips.push_back(strip.clone());
                    idx++; continue;
               }

               if(debug) printf(" ---------- [Thresholding Strip %d] ---------- \r\n\tMax = %d, Mean = %.1lf, Std = %.1lf\r\n", idx,tmpMax,tmpMean,tmpStd);
               double maxValRatio = (double) uMax/255.0;
               double relRatio = (double)(tmpMax-tmpStd)/(double)uMax;
               double relRatio2 = (tmpMean)/(double)(tmpMax);
               if(debug) printf("\tRatios: %.3lf, %.3lf, %.3lf\r\n", maxValRatio, relRatio, relRatio2);

               double tmpGain;
               if(relRatio >= 0.4) tmpGain = relRatio + relRatio2;
               else tmpGain = 1.0 - (relRatio + relRatio2);

               int thresh = int(threshs[idx] * tmpGain * tmpMax);
               if(debug) printf("\tGain = %.2lf, Thresh = %d\r\n", tmpGain,thresh);

               cv::Mat tmpStrip;
               threshold(strip, tmpStrip, thresh, 255,cv::THRESH_TOZERO);
               pStrips.push_back(tmpStrip.clone());
               idx++;
          }
          err = merge_strips(pStrips, &filtered, false);
     } else{
          if(verbose) printf("[WARNING] preprocess_umap_stripping() --- Could not properly strip input umap given the stripping parameters. Returning unfiltered umap.\r\n");
          int wholeThresh = int(0.25*uMax);
          cv::threshold(imgCopy, filtered, wholeThresh, 255, cv::THRESH_TOZERO);
     }

     if(output) *output = filtered;
}
cv::Mat preprocess_vmap_stripping(const cv::Mat& input, vector<float>* thresholds, bool verbose, bool debug){
     cv::Mat output;
     preprocess_vmap_stripping(input, &output, thresholds, verbose, debug);
     return output;
}
void preprocess_vmap_stripping(const cv::Mat& input, cv::Mat* output,
     vector<float>* thresholds, bool verbose, bool debug)
{
     cv::Mat filtered;
     if(input.empty()) return;
     cv::Mat imgCopy = input.clone();

     vector<float> threshs;
     if(thresholds) threshs = (*thresholds);
     if(threshs.empty()) threshs = vector<float>{0.3, 0.3,0.25,0.4};
     int nThreshs = threshs.size();

     if(verbose){
          std::string threshStr = vector_str(threshs, ", ");
          printf("[INFO] preprocess_vmap_stripping() --- Using %d thresholds = [%s]\r\n", (int)threshs.size(), threshStr.c_str());
     }

     double vMin, vMax;
     cv::minMaxLoc(imgCopy, &vMin, &vMax);

     float ratio = (float)(imgCopy.cols) / 255.0;
     int nStrips = int(ceil(ratio * nThreshs)) - 1;
     if(debug) printf("vMax, sizeRatio, nThreshs: %d, %.2f, %d\r\n", (int) vMax, ratio, nStrips);

     std::vector<cv::Mat> strips;
     int err = strip_image(imgCopy, &strips, nStrips, true);

     int idx = 0;
     std::vector<cv::Mat> pStrips;
     if(err >= 0){
          for(cv::Mat strip : strips){
               double stripMin, stripMax;
               cv::Scalar stripMean, stripStddev;
               cv::minMaxLoc(strip, &stripMin, &stripMax);
               cv::meanStdDev(strip, stripMean, stripStddev);

               int tmpMax = (int) stripMax;
               double tmpMean = (double) stripMean[0];
               double tmpStd = (double) stripStddev[0];
               if(tmpMean == 0.0){
                    pStrips.push_back(strip.clone());
                    idx++; continue;
               }

               if(debug) printf(" ---------- [Thresholding Strip %d] ---------- \r\n\tMax = %d, Mean = %.1lf, Std = %.1lf\r\n", idx,tmpMax,tmpMean,tmpStd);
               double maxValRatio = (double) vMax/255.0;
               double relRatio = (double)(tmpMax-tmpStd)/(double)vMax;
               double relRatio2 = (tmpMean)/(double)(tmpMax);
               if(debug) printf("\tRatios: %.3lf, %.3lf, %.3lf\r\n", maxValRatio, relRatio, relRatio2);

               double tmpGain;
               if(relRatio >= 0.4) tmpGain = relRatio + relRatio2;
               else tmpGain = 1.0 - (relRatio + relRatio2);

               int thresh = int(threshs[idx] * tmpGain * tmpMax);
               if(debug) printf("\tGain = %.2lf, Thresh = %d\r\n", tmpGain,thresh);

               cv::Mat tmpStrip;
               threshold(strip, tmpStrip, thresh, 255,cv::THRESH_TOZERO);
               pStrips.push_back(tmpStrip.clone());
               idx++;
          }
          err = merge_strips(pStrips, &filtered);
     }else{
          if(verbose) printf("[WARNING] preprocess_vmap_stripping() --- Could not properly strip input vmap given the stripping parameters. Returning unfiltered vmap.\r\n");
          int wholeThresh = int(0.25*vMax);
          cv::threshold(imgCopy, filtered, wholeThresh, 255, cv::THRESH_TOZERO);
     }

     if(output) *output = filtered;
}
cv::Mat preprocess_umap_sobelized(const cv::Mat& umap, int thresh_pre_sobel,
     int thresh_sobel_preprocess, int thresh_sobel_postprocess, int dilate_size,
     int blur_size, vector<int> kernel_multipliers, cv::Mat* keep_mask, cv::Mat* sobel_raw,
     cv::Mat* sobel_preprocessed, cv::Mat* sobel_dilated, cv::Mat* sobel_blurred)
{
     cv::Mat output;
     if(umap.empty()) return output;
     int kernel_x_multiplier = 10, kernel_y_multiplier = 2;
     if(!kernel_multipliers.empty()){
          kernel_x_multiplier = kernel_multipliers[0];
          kernel_y_multiplier = kernel_multipliers[1];
     }
     // Create output containers for debugging stages during processing
     cv::Mat umapSobel, sobelThreshed, sobelDilated, sobelBlurred, keepMask;

     // Create Sobel-ized Umap
     cv::Mat rawUmap, umapThreshed;
     umap.convertTo(rawUmap, CV_64F);
     threshold(rawUmap, umapThreshed, float(thresh_pre_sobel), 255, cv::THRESH_TOZERO);
     cv::Sobel(umapThreshed, umapSobel, CV_64F, 0, 1, 3);
     if(sobel_raw) *sobel_raw = umapSobel.clone();

     // Threshold sobel-ized umap before processing
     double minVal, maxVal;
     cv::minMaxLoc(umapSobel, &minVal, &maxVal);
     umapSobel = umapSobel * (255.0/maxVal);
     cv::convertScaleAbs(umapSobel, umapSobel, 1, 0);
     threshold(umapSobel, sobelThreshed, float(thresh_sobel_preprocess), 255, cv::THRESH_TOZERO);
     if(sobel_preprocessed) *sobel_preprocessed = sobelThreshed.clone();

     // Process Sobel-ized Umap into a "keep" mask
     cv::Mat dilate_element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
          cv::Size(dilate_size * kernel_x_multiplier, dilate_size * kernel_y_multiplier)
     );
     cv::dilate(sobelThreshed, sobelDilated, dilate_element, cv::Point(-1,-1), 1);
     if(sobel_dilated) *sobel_dilated = sobelDilated.clone();

     cv::blur(sobelDilated, sobelBlurred,
          cv::Size(blur_size * kernel_x_multiplier, blur_size * kernel_y_multiplier)
     );
     if(sobel_blurred) *sobel_blurred = sobelBlurred.clone();

     threshold(sobelBlurred, keepMask, 0, 255, cv::THRESH_BINARY);
     if(keep_mask) *keep_mask = keepMask.clone();

     // Filter the input umap using mask created from sobel images, and threshold result
     umap.copyTo(output, keepMask);
     threshold(output, output, float(thresh_sobel_postprocess), 255, cv::THRESH_TOZERO);
     return output.clone();
}
cv::Mat preprocess_umap_sobelized(const cv::Mat& umap, int thresh_pre_sobel,
     int thresh_sobel_preprocess, int thresh_sobel_postprocess, int dilate_size,
     int blur_size, vector<int> kernel_multipliers, VboatsProcessingImages* image_debugger)
{
     cv::Mat processed;
     if(image_debugger){
          cv::Mat* debug_image = nullptr;
          cv::Mat* debug_image2 = nullptr;
          cv::Mat* debug_image3 = nullptr;
          cv::Mat* debug_image4 = nullptr;
          cv::Mat* debug_image5 = nullptr;
          if(image_debugger->visualize_umap_keep_mask) debug_image = new cv::Mat();
          if(image_debugger->visualize_umap_sobel_raw) debug_image2 = new cv::Mat();
          if(image_debugger->visualize_umap_sobel_preprocessed) debug_image3 = new cv::Mat();
          if(image_debugger->visualize_umap_sobel_dilated) debug_image4 = new cv::Mat();
          if(image_debugger->visualize_umap_sobel_blurred) debug_image5 = new cv::Mat();

          processed = preprocess_umap_sobelized(umap, thresh_pre_sobel,
               thresh_sobel_preprocess, thresh_sobel_postprocess,
               dilate_size, blur_size, kernel_multipliers,
               debug_image,  // keep_mask visualization
               debug_image2,  // sobel_raw visualization
               debug_image3,  // sobel_preprocessed visualization
               debug_image4,  // sobel_dilated visualization
               debug_image5   // sobel_blurred visualization
          );

          if(image_debugger->visualize_umap_keep_mask){
               image_debugger->set_umap_sobelized_keep_mask(*debug_image);
               delete debug_image;
          }
          if(image_debugger->visualize_umap_sobel_raw){
               image_debugger->set_umap_sobelized_raw(*debug_image2);
               delete debug_image2;
          }
          if(image_debugger->visualize_umap_sobel_preprocessed){
               image_debugger->set_umap_sobelized_pre_filtering(*debug_image3);
               delete debug_image3;
          }
          if(image_debugger->visualize_umap_sobel_dilated){
               image_debugger->set_umap_sobelized_dilated(*debug_image4);
               delete debug_image4;
          }
          if(image_debugger->visualize_umap_sobel_blurred){
               image_debugger->set_umap_sobelized_blurred(*debug_image5);
               delete debug_image5;
          }
     } else{
          processed = preprocess_umap_sobelized(umap, thresh_pre_sobel,
               thresh_sobel_preprocess, thresh_sobel_postprocess,
               dilate_size, blur_size, kernel_multipliers,
               nullptr,  // keep_mask visualization
               nullptr,  // sobel_raw visualization
               nullptr,  // sobel_preprocessed visualization
               nullptr,  // sobel_dilated visualization
               nullptr   // sobel_blurred visualization
          );
     }
     return processed;
}
cv::Mat preprocess_vmap_sobelized(const cv::Mat& vmap, int thresh_sobel, int blur_size, vector<int> kernel_multipliers){
     cv::Mat output;
     if(vmap.empty()) return output;

     int kernel_x_multiplier = 1, kernel_y_multiplier = 1;
     if(!kernel_multipliers.empty()){
          kernel_x_multiplier = kernel_multipliers[0];
          kernel_y_multiplier = kernel_multipliers[1];
     }

     cv::Mat rawVmap, blurVmap, sobel;
     vmap.convertTo(rawVmap, CV_64F);
     cv::blur(rawVmap, blurVmap,
          cv::Size(blur_size * kernel_x_multiplier, blur_size * kernel_y_multiplier)
     );
     cv::Sobel(blurVmap, sobel, CV_64F, 0, 1, 3);

     double minVal, maxVal;
     cv::minMaxLoc(sobel, &minVal, &maxVal);
     sobel = sobel * (255.0/maxVal);
     cv::convertScaleAbs(sobel, sobel, 1, 0);
     threshold(sobel, output, float(thresh_sobel), 255, cv::THRESH_TOZERO);
     return output.clone();
}
cv::Mat postprocess_vmap_sobelized(const cv::Mat& vmap, const cv::Mat& preprocessed_sobel,
     int thresh_preprocess, int thresh_postprocess, int blur_size, vector<int> kernel_multipliers,
     cv::Mat* sobel_threshed, cv::Mat* sobel_blurred, cv::Mat* keep_mask
){
     cv::Mat output;
     if(vmap.empty()) return output;
     if(preprocessed_sobel.empty()) return vmap;

     int kernel_x_multiplier = 1, kernel_y_multiplier = 1;
     if(!kernel_multipliers.empty()){
          kernel_x_multiplier = kernel_multipliers[0];
          kernel_y_multiplier = kernel_multipliers[1];
     }
     // Create output containers for debugging stages during processing
     cv::Mat sobelThreshed, sobelBlurred, keepMask;

     // Threshold sobel-ized vmap created from pre-processing before further processing of sobel-ized vmap
     threshold(preprocessed_sobel, sobelThreshed, float(thresh_preprocess), 255, cv::THRESH_TOZERO);
     if(sobel_threshed) *sobel_threshed = sobelThreshed.clone();

     // Blur sobel-ized vmap for creating a conservative "keep" mask
     cv::blur(sobelThreshed, sobelBlurred,
          cv::Size(blur_size * kernel_x_multiplier, blur_size * kernel_y_multiplier)
     );
     if(sobel_blurred) *sobel_blurred = sobelBlurred.clone();

     // Threshold processed sobel-ized vmap into a "keep" mask
     threshold(sobelBlurred, keepMask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
     cv::bitwise_not(keepMask,keepMask);
     if(keep_mask) *keep_mask = keepMask.clone();

     // Filter the original vmap using the "keep" mask, and threshold result
     vmap.copyTo(output, keepMask);
     threshold(output, output, float(thresh_postprocess), 255, cv::THRESH_TOZERO);
     return output.clone();
}
cv::Mat postprocess_vmap_sobelized(const cv::Mat& vmap, const cv::Mat& preprocessed_sobel,
     int thresh_preprocess, int thresh_postprocess, int blur_size, vector<int> kernel_multipliers,
     VboatsProcessingImages* image_debugger
){
     cv::Mat processed;
     if(image_debugger){
          cv::Mat* debug_image = nullptr;
          cv::Mat* debug_image2 = nullptr;
          cv::Mat* debug_image3 = nullptr;
          if(image_debugger->visualize_vmap_post_sobel_threshed) debug_image = new cv::Mat();
          if(image_debugger->visualize_vmap_post_sobel_blurred) debug_image2 = new cv::Mat();
          if(image_debugger->visualize_vmap_post_keep_mask) debug_image3 = new cv::Mat();

          processed = postprocess_vmap_sobelized(vmap, preprocessed_sobel,
               thresh_preprocess, thresh_postprocess, blur_size, kernel_multipliers,
               debug_image,  // sobel_threshed visualization
               debug_image2,  // sobel_blurred visualization
               debug_image3   // keep_mask visualization
          );

          if(image_debugger->visualize_vmap_post_sobel_threshed){
               image_debugger->set_vmap_sobelized_thresholded(*debug_image);
               delete debug_image;
          }
          if(image_debugger->visualize_vmap_post_sobel_blurred){
               image_debugger->set_vmap_sobelized_postprocessed_blurred(*debug_image2);
               delete debug_image2;
          }
          if(image_debugger->visualize_vmap_post_keep_mask){
               image_debugger->set_vmap_sobelized_postprocessed_keep_mask(*debug_image3);
               delete debug_image3;
          }
     } else{
          processed = postprocess_vmap_sobelized(vmap, preprocessed_sobel,
               thresh_preprocess, thresh_postprocess, blur_size, kernel_multipliers,
               nullptr,  // sobel_threshed visualization
               nullptr,  // sobel_blurred visualization
               nullptr   // keep_mask visualization
          );
     }
     return processed;

}

/** =========================
*   Ground Line Extraction
* =========================== */
void get_hough_line_params(const float& rho, const float& theta, float* slope, int* intercept){
     float a = cos(theta),    b = sin(theta);
     float x0 = a*rho,        y0 = b*rho;
     int x1 = int(x0 - 1000.0 * b);
     int y1 = int(y0 + 1000.0 * a);
     int x2 = int(x0 + 1000.0 * b);
     int y2 = int(y0 - 1000.0 * a);
     float rise = (float)(y2 - y1);
     float run = (float)(x2 - x1);
     float _m = rise / run;
     int _intercept = y1 - int(_m * x1);
     if(slope) *slope = _m;
     if(intercept) *intercept = _intercept;
}
int estimate_ground_line_coefficients(const vector<cv::Vec2f>& lines, float* best_slope,
     int* best_intercept, float* worst_slope, int* worst_intercept,
     double gnd_deadzone, double minDeg, double maxDeg,
     bool verbose, bool debug_timing
){
     double t;
     vector<cv::Vec2f> buf;
     int nLines = lines.size();
     if(nLines <= 0) return -1;
     if(debug_timing) t = (double)cv::getTickCount();

     float dAng = gnd_deadzone * M_DEG2RAD;
     float minAng = (minDeg * M_DEG2RAD) + (CV_PI/2.0);
     float maxAng = (maxDeg * M_DEG2RAD) + (CV_PI/2.0);
     float sumRad = 0.0;
     float tmpRho, tmpAng;
     if(verbose) printf("[INFO] estimate_ground_line_coefficients() ---- Filtering lines using limits (deg): Min=%.2f -- Max=%.2f\r\n", minAng*M_RAD2DEG, maxAng*M_RAD2DEG);
     for(int i = 0; i < nLines; i++){
          tmpRho = lines[i][0];
          tmpAng = lines[i][1];
          if(verbose){
               if(tmpAng > 0) printf("[INFO] estimate_ground_line_coefficients() ---- \tFiltering Line[%d] -- Angle=%.2f (deg)\r\n", i, tmpAng*M_RAD2DEG);
          }
          if(tmpAng >= minAng && tmpAng <= maxAng){
               sumRad += tmpAng;
               buf.push_back(cv::Vec2f(tmpRho, tmpAng));
          }
     }

     int num = 0, idx = 0;
     int tmpB = 0, bestB = 0, worstB = 0;
     float tmpM = 0.0, bestM = 0.0, worstM = 0.0;
     float avgAng = sumRad / float(buf.size());
     float minAvg = avgAng - dAng;
     float maxAvg = avgAng + dAng;
     if(verbose) printf("[INFO] estimate_ground_line_coefficients() ---- Angle Limits (deg): Min=%.2f -- Avg=%.2f -- Max=%.2f\r\n", (minAvg*M_RAD2DEG)-90.0, (avgAng*M_RAD2DEG)-90.0, (maxAvg*M_RAD2DEG)-90.0);
     for(cv::Vec2f tmpLine : buf){
          idx++;
          if(verbose) printf("[INFO] estimate_ground_line_coefficients() ---- Line[%d] -- Angle=%.2f (deg)\r\n", idx, (tmpLine[1]*M_RAD2DEG)-90.0);
          if(tmpLine[1] >= minAvg && tmpLine[1] <= maxAvg){
               get_hough_line_params(tmpLine[0],tmpLine[1], &tmpM, &tmpB);
               if(tmpB > bestB){ bestM = tmpM; bestB = tmpB; }
               if(tmpB < worstB){ worstM = tmpM; worstB = tmpB; }
               num++;
          }
     }
     if(verbose) printf("[INFO] estimate_ground_line_coefficients() ---- ground line best(m = %.2f, b = %d) -- worst(m = %.2f, b = %d) estimated found from %d / %d hough lines\r\n", bestM, bestB, worstM, worstB, num,nLines);
     if(debug_timing){
          t = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
          printf("[INFO] estimate_ground_line_coefficients() ---- took %.4lf ms (%.2lf Hz)\r\n", t*1000.0, (1.0/t));
     }

     float slope, otherSlope;  int intercept, otherIntercept;
     bool bestValid = (bestB != 0) && (bestM != 0);
     bool worstValid = (worstB != 0) && (worstM != 0);
     if(!bestValid && worstValid){
          slope = worstM;
          intercept = worstB;
          otherSlope = bestM;
          otherIntercept = bestB;
     } else{
          slope = bestM;
          intercept = bestB;
          otherSlope = worstM;
          otherIntercept = worstB;
     }

     if(best_slope) *best_slope = slope;
     if(best_intercept) *best_intercept = intercept;
     if(worst_slope) *worst_slope = otherSlope;
     if(worst_intercept) *worst_intercept = otherIntercept;
     return num;
}
int find_ground_lines(const cv::Mat& vmap, cv::Mat* found_lines, int hough_thresh, bool verbose){
     if(vmap.empty()) return -1;
     cv::Mat _lines;
     cv::HoughLines(vmap, _lines, 1, CV_PI/180, hough_thresh);
     int n = _lines.size().height;
     int m = _lines.size().width;
     int c = _lines.channels();
     if(verbose) printf("[INFO] find_ground_lines() ---- Found %d, %d hough lines in image.\r\n", n, m);
     if(found_lines) *found_lines = _lines;
     return n;
}
int find_ground_lines(const cv::Mat& vmap, cv::Mat* rhos, cv::Mat* thetas, int hough_thresh, bool verbose){
     if(vmap.empty()) return -1;
     cv::Mat _lines;
     cv::HoughLines(vmap, _lines, 1, CV_PI/180, hough_thresh);
     int n = _lines.size().height;
     int m = _lines.size().width;
     int c = _lines.channels();
     if(verbose) printf("[INFO] find_ground_lines() ---- Found %d hough lines in image.\r\n", n);

     cv::Mat rs(n, 1, CV_32FC1);
     cv::Mat angs(n, 1, CV_32FC1);
     cv::Mat out[] = { rs, angs };
     int from_to[] = { 0,0, 1,1};
     cv::mixChannels( &_lines, 1, out, 2, from_to, 2 );

     if(rhos) *rhos = out[0];
     if(thetas) *thetas = out[1];
     return n;
}
int find_ground_lines(const cv::Mat& vmap, vector<cv::Vec2f>* found_lines, int hough_thresh, bool verbose){
     if(vmap.empty()) return -1;
     vector<cv::Vec2f> _lines;
     cv::HoughLines(vmap, _lines, 1, CV_PI/180, hough_thresh);
     int n = _lines.size();
     if(verbose) printf("[INFO] find_ground_lines() ---- Found %d hough lines in image.\r\n", n);
     if(found_lines) *found_lines = _lines;
     return n;
}

bool find_ground_line(const cv::Mat& vmap, std::vector<float>* best_coeffs,
     double minDeg, double maxDeg, double gnd_deadzone, int hough_thresh,
     bool verbose, bool debug_timing)
{
     float gndM; int gndB;
     bool gndPresent = find_ground_line(vmap, &gndM,&gndB, minDeg, maxDeg, gnd_deadzone, hough_thresh, verbose, debug_timing);

     std::vector<float> linear_coefficients;
     if(gndPresent) linear_coefficients = std::vector<float>{gndM, (float) gndB};
     if(best_coeffs) *best_coeffs = linear_coefficients;
     return gndPresent;
}
bool find_ground_line(const cv::Mat& vmap, float* best_slope, int* best_intercept,
     double minDeg, double maxDeg, double gnd_deadzone, int hough_thresh,
     bool verbose, bool debug_timing
){
     if(vmap.empty()) return false;

     double t;
     if(debug_timing) t = (double)cv::getTickCount();

     std::vector<cv::Vec2f> lines;
     int nLines = find_ground_lines(vmap, &lines, hough_thresh);
     if(nLines <= 0) return false;

     float m, badm;
     int b, badb;
     int nUsed = estimate_ground_line_coefficients(lines, &m, &b, &badm, &badb, gnd_deadzone, minDeg, maxDeg);
     if(nUsed <= 0) return false;

     if(debug_timing){
          t = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
          printf("[INFO] find_ground_line() ---- took %.4lf ms (%.2lf Hz)\r\n", t*1000.0, (1.0/t));
     }
     if(best_slope) *best_slope = m;
     if(best_intercept) *best_intercept = b;
     return true;
}

/** =========================
*   U-Map Contour Extraction
* =========================== */
void extract_contour_bounds(const vector<cv::Point>& contour,
     vector<int>* xbounds, vector<int>* dbounds, bool verbose)
{
     cv::Rect tmpRect = cv::boundingRect(contour);
     vector<int> _xbounds = {tmpRect.x, tmpRect.x + tmpRect.width};
     vector<int> _dbounds = {tmpRect.y, tmpRect.y + tmpRect.height};

     if(xbounds) *xbounds = _xbounds;
     if(dbounds) *dbounds = _dbounds;
}

void find_contours(const cv::Mat& umap, vector<vector<cv::Point>>* filtered_contours,
     int filter_method, float min_threshold, float max_threshold, cv::Point* offset,
     vector<cv::Vec4i>* filtered_hierarchies, vector<vector<cv::Point>>* all_contours,
     vector<cv::Vec4i>* all_hierarchies, bool verbose, bool debug, bool debug_timing)
{
     if(umap.empty()) return;

     double t;
     if(debug_timing) t = (double)cv::getTickCount();

     cv::Mat image;
     if(umap.channels() <= 3) image = umap;
     else cv::cvtColor(umap, image, cv::COLOR_BGR2GRAY);

     int offx, offy;
     if(offset){
          offx = offset->x;
          offy = offset->y;
     } else{ offx = 0; offy = 0; }
     if(debug) printf("[DEBUG] find_contours() ---- Using contour offsets (x,y): %d, %d\r\n", offx, offy);

     vector<cv::Vec4i> hierarchy;
     vector< vector<cv::Point> > contours;
     cv::findContours(image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(offx, offy));
     if(debug) printf("[DEBUG] find_contours() ---- Found %d contours\r\n", (int) contours.size());

     vector<cv::Vec4i> filtHierarchy;
     vector<vector<cv::Point>> filtContours;
     double tmpVal, tmpArea, tmpLength, minVal = 1000, maxVal = 0;
     if(filter_method > 0){
          for(int i = 0; i < (int) contours.size(); i++){
               vector<cv::Point> contour = contours[i];
               cv::Vec4i hier = hierarchy[i];
               if(filter_method == 1){            /** Perimeter-based filtering */
                    tmpVal = cv::arcLength(contour,true);
                    if(tmpVal < min_threshold) continue;
                    if(max_threshold > 0){ if(tmpVal > max_threshold) continue; }
                    filtContours.push_back(contour);
                    filtHierarchy.push_back(hier);
                    if(tmpVal < minVal) minVal = tmpVal;
                    if(tmpVal > maxVal) maxVal = tmpVal;
               }else if(filter_method == 2){      /** Area-based filtering */
                    tmpVal = cv::contourArea(contour);
                    if(tmpVal < min_threshold) continue;
                    if(max_threshold > 0){ if(tmpVal > max_threshold) continue; }
                    filtContours.push_back(contour);
                    filtHierarchy.push_back(hier);
                    if(tmpVal < minVal) minVal = tmpVal;
                    if(tmpVal > maxVal) maxVal = tmpVal;
               }
          }
     } else filtContours = contours;
     if(debug){
          printf("[DEBUG] find_contours() ---- %d contours remaining after filtering.\r\n", (int) filtContours.size());
          printf("[DEBUG] find_contours() ---- Filtered Contours Extremes (max, min): %.2lf, %.2lf\r\n",maxVal,minVal);
     }

     if(debug_timing){
          t = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
          printf("[INFO] find_contours() ---- took %.4lf ms (%.2lf Hz) to find %d / %d contours\r\n", t*1000.0, (1.0/t), (int) filtContours.size(), (int) contours.size());
     }
     if(filtered_contours) *filtered_contours = filtContours;
     if(filtered_hierarchies) *filtered_hierarchies = filtHierarchy;
     if(all_contours) *all_contours = contours;
     if(all_hierarchies) *all_hierarchies = hierarchy;
}

/** =========================
*     Obstacle Extraction
* =========================== */
int obstacle_search_disparity(const cv::Mat& vmap, const vector<int>& xLimits,
     vector<int>* yLimits, int* pixel_thresholds, int* window_size,
     std::vector<float> line_params, vector<cv::Rect>* obs_windows,
     bool verbose, bool debug, bool debug_timing)
{
     if(vmap.empty()) return -1;

     double t;
     if(debug_timing) t = (double)cv::getTickCount();

     cv::Mat img;
     if(vmap.channels() < 3) img = vmap;
     else cv::cvtColor(vmap, img, cv::COLOR_BGR2GRAY);
     int h = img.rows, w = img.cols;

     int xmin = xLimits.at(0);
     int xmax = xLimits.at(1);
     int pxlMin = 3, pxlMax = 30;
     if(pixel_thresholds){ pxlMin = pixel_thresholds[0], pxlMax = pixel_thresholds[1]; }

     int yk = 0;
     int xk = (int) xmin;
     int xmid = (int)((float)(xmax + xmin) / 2.0);
     int dWy = 10, dWx = abs(xmid - xmin);

     int yf = h, yf_edge = h;
     if(!line_params.empty()){
          yf = (int)(xmid * line_params[0] + line_params[1]);
          yf_edge = (int)((xmin) * line_params[0] + line_params[1]);
          if(debug) printf("Ground Line Coefficients - slope = %.2f, intercept = %d\r\n", line_params[0], (int)line_params[1]);
     }
     if(window_size){
          dWx = (int)((float) window_size[0] / 2.0);
          dWy = (int)((float) window_size[1] / 2.0);
     }
     if(dWx <= 2){
          if(xk <= 5) dWx = 1;
          else dWx = 2;
     }
     if(xk+2*dWx >= w){
          if(debug) printf("Problem xk = %d\r\n", xk);
          xk = (w-1)-2*dWx;
          if(debug) printf("New xk = %d\r\n", xk);
     }
     cv::Rect searchRoi = cv::Rect(xk, yk, dWx*2, dWy*2);

     if(debug){
          printf("Input Image [%d x %d]\r\n", w,h);
          printf("X limits = [%d, %d] --- Y limits = [%d, %d] --- Pixel limits = [%d, %d] --- Window Size = [%d, %d]\r\n", xmin,xmax,yk, yf,pxlMin,pxlMax,dWx,dWy);
          printf("Object Search Window [%d x %d] -- Starting Location (%d, %d)\r\n", dWx,dWy,xk, yk);
     }

     vector<int> _yLimits;
     vector<vector<cv::Point>> windows;
     vector<cv::Rect> obstacle_windows;
     int count = 0, nWindows = 0, prev_yk = 0;
     bool flag_done = false, try_last_search = false;
     bool flag_found_start = false, flag_hit_limits = false;
     while(!flag_done){
          if(xk >= w){
               flag_hit_limits = true;
               if(debug) printf("[INFO] Reached max image width.\r\n");
          }
          if((yk >= yf) && (!line_params.empty())){
               flag_hit_limits = true;
               if(debug) printf("[INFO] Reached Estimated Ground line at y = %d.\r\n", yk);
          } else if((yk >= yf_edge) && (!line_params.empty())){
               flag_hit_limits = true;
               if(debug) printf("[INFO] Search Window Edge Reached Estimated Ground line at y = %d.\r\n", yk);
          } else if(yk + dWy >= h){
               flag_hit_limits = true;
               if(debug) printf("[INFO] Reached max image height.\r\n");
          }
          if(flag_hit_limits) flag_done = true;
          if(count != 0){
               // Slide window from previousy found center (Clip at image edges)
               // Update vertical [Y] window edges
               if(yk - 2*dWy < 0) yk = 0;
               if(yk + 2*dWy >= h) yk = h - 2*dWy;
               // Update horizontal [X] window edges
               if(xk - 2*dWx < 0) xk = dWx;
               if(xk + 2*dWx >= w) xk = w - 2*dWx;
               searchRoi.y = yk;
          }

          if(debug) std::cout << "searchRoi: " << searchRoi << std::endl;
          cv::Mat roi = img(searchRoi);
          int nPxls = cv::countNonZero(roi);
          if(verbose) printf("Current Window [%d] ----- Center = (%d, %d) ----- %d of good pixels\r\n",count,xk,yk,nPxls);

          if(nPxls >= pxlMax){
               if(nWindows == 0){
                    _yLimits.push_back(yk);
                    if(flag_hit_limits){
                         flag_done = false;
                         if(debug) printf("\tTrying Last Ditch Search...\r\n");
                    }
               } else{ _yLimits.push_back(yk+2*dWy); try_last_search = false; }

               vector<cv::Point> tmpWindow = {
                    cv::Point(xk,yk),
                    cv::Point(xk+2*dWx,yk+2*dWy)
               };
               windows.push_back(tmpWindow);
               nWindows++;
               prev_yk = yk;
               flag_found_start = true;
          } else if((nPxls <= pxlMin) && (flag_found_start)){
               if(debug) printf("Exiting - minimum threshold found...\r\n");
               flag_done = true;
               _yLimits.push_back(yk);
          }
          obstacle_windows.push_back(searchRoi);
          yk = yk + 2*dWy;
          count++;
     }
     if(debug){
          if(_yLimits.size() == 0) printf("[%d] Good Windows --- Ylimits = %d - %d\r\n", nWindows, 0, 0);
          else printf("[%d] Good Windows --- Ylimits = %d - %d\r\n", nWindows, _yLimits[0], _yLimits.back());
     }

     if(debug_timing){
          t = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
          printf("[INFO] obstacle_search_disparity() ---- took %.4lf ms (%.2lf Hz) to find %d good windows\r\n", t*1000.0, (1.0/t), nWindows);
     }
     if(yLimits) *yLimits = _yLimits;
     if(obs_windows) *obs_windows = obstacle_windows;
     return nWindows;
}
int find_obstacles_disparity(const cv::Mat& vmap,
     const std::vector< std::vector<cv::Point> >& contours, std::vector<float> line_params,
     std::vector<Obstacle>* found_obstacles, std::vector< vector<cv::Rect> >* obstacle_windows,
     bool verbose, bool debug_timing)
{
     if(vmap.empty()) return -1;

     bool gndPresent;
     if(!line_params.empty()){
          gndPresent = true;
          if(verbose) printf("Ground Line Coefficients - slope = %.2f, intercept = %d\r\n", line_params[0], (int)line_params[1]);
     } else{
          gndPresent = false;
          if(verbose) printf("No Ground Found\r\n");
     }

     double t;
     if(debug_timing) t = (double)cv::getTickCount();

     int nObs = 0;
     vector<int> xLims;
     vector<int> dLims;
     vector<int> yLims;
     vector<Obstacle> obs;
     vector< vector<cv::Rect> > windows;
     for(int i = 0; i < (int) contours.size(); i++){
          vector<cv::Rect> obWindows;
          vector<cv::Point> contour = contours[i];
          extract_contour_bounds(contour,&xLims, &dLims);

          int nWins;
          if(obstacle_windows) nWins = obstacle_search_disparity(vmap,dLims, &yLims, nullptr, nullptr, line_params, &obWindows, VERBOSE_OBS_SEARCH);
          else nWins = obstacle_search_disparity(vmap,dLims, &yLims, nullptr, nullptr, line_params, nullptr, VERBOSE_OBS_SEARCH);

          if(nWins == 0) continue;
          if((yLims.size() <= 2) && (gndPresent)){
               if(verbose) printf("[INFO] Found obstacle with zero height. Skipping...\r\n");
               continue;
          } else if(yLims.size() <= 1){
               if(verbose) printf("[INFO] Found obstacle with zero height. Skipping...\r\n");
               continue;
          } else if(yLims[0] == yLims.back()){
               if(verbose) printf("[INFO] Found obstacle with zero height. Skipping...\r\n");
               continue;
          }else{
               if(verbose) printf("[INFO] Adding Obstacle (%d y limits)...\r\n", (int)yLims.size());
               vector<cv::Point> pts = {cv::Point(xLims[0],yLims[0]), cv::Point(xLims[1],yLims.back())};
               obs.push_back(Obstacle(pts,dLims));
               if(obstacle_windows) windows.push_back(obWindows);
               nObs++;
          }
          if(verbose) printf(" --------------------------- \r\n");
     }
     if(debug_timing){
          t = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
          printf("[INFO] find_obstacles() ---- took %.4lf ms (%.2lf Hz) to find %d obstacles\r\n", t*1000.0, (1.0/t), (int)obs.size());
     }
     if(found_obstacles) *found_obstacles = obs;
     if(obstacle_windows) *obstacle_windows = windows;
     return nObs;
}
int find_obstacles_disparity(const cv::Mat& vmap,
     const std::vector< std::vector<cv::Point> >& contours, std::vector<float> line_params,
     std::vector<Obstacle>* found_obstacles, VboatsProcessingImages* image_debugger,
     bool verbose, bool debug_timing)
{
     int err = 0;
     if(image_debugger && image_debugger->visualize_extracted_object_windows){
          VboatsProcessingImages debugger = (*image_debugger);
          err = find_obstacles_disparity(vmap, contours, line_params, found_obstacles,
               &debugger.vmap_object_search_regions,
               verbose, debug_timing
          );
     } else{
          std::vector< vector<cv::Rect> >* obstacle_windows = nullptr;
          err = find_obstacles_disparity(vmap, contours, line_params, found_obstacles,
               obstacle_windows, verbose, debug_timing
          );
     }
     return err;
}

/** =========================
*  Depth Filtering Strategies
* =========================== */
int filter_depth_using_ground_line(const cv::Mat& depth, const cv::Mat& disparity,
     const cv::Mat& vmap, std::vector<float> line_params, cv::Mat* filtered_image,
     std::vector<int> line_intercept_offsets, cv::Mat* keep_mask, bool verbose, bool debug_timing)
{
     cv::Mat filtered_depth;
     if(depth.empty()){
          if(verbose) printf("[WARNING] remove_ground() --- Depth Image is empty, skipping ground removal.\r\n");
          return -1;
     }
     if(vmap.empty()){
          if(verbose) printf("[WARNING] remove_ground() --- Vmap input Image is empty, skipping ground removal.\r\n");
          if(filtered_image) *filtered_image = depth.clone();
          return -2;
     }
     if(disparity.empty()){
          if(verbose) printf("[WARNING] remove_ground() --- Disparity input Image is empty, skipping ground removal.\r\n");
          if(filtered_image) *filtered_image = depth.clone();
          return -3;
     }
     if(line_params.empty()){
          if(verbose) printf("[WARNING] remove_ground() --- Linear Coefficients for Ground Line are unknown, skipping ground removal.\r\n");
          if(filtered_image) *filtered_image = depth.clone();
          return -4;
     }
     cv::Mat originalDepth, gndLineRefImg, keepMaskRefImg;
     keepMaskRefImg = disparity.clone();
     gndLineRefImg = vmap.clone();
     originalDepth = depth.clone();

     // Create a blank "keep" mask whose pixels are filled only if
     cv::Mat removeMask = cv::Mat::zeros(disparity.size(), CV_8UC1);

     // Y-intercept offsets to use either a conservative (upper offset) or
     // a liberal (lower offset) alternative to the original estimated ground line
     int lower_line_intercept_offset = 0;    // Use this for a more liberal filtering of the ground
     int upper_line_intercept_offset = 0;    // Use this for a more conservative filtering of the ground
     if(!line_intercept_offsets.empty()){
          lower_line_intercept_offset = line_intercept_offsets[0];
          upper_line_intercept_offset = line_intercept_offsets[1];
     }
     // Calculate ROI limits based on estimated ground line coefficients
     float slope = line_params[0];
     int intercept = (int) line_params[1];
     int lower_intercept = intercept + lower_line_intercept_offset;
     int upper_intercept = intercept - upper_line_intercept_offset;
     // NOTE: I forget the original purpose for these variables, but keeping them here for now
     int lower_intercept_final = (int)(vmap.cols * slope + (lower_intercept));
     int upper_intercept_final = (int)(vmap.cols * slope + (upper_intercept));
     // Ensure the y-intercept value is non-zero and use that as the starting row for searching pixels
     int initial_row;
     if(intercept < 0) initial_row = 0;
     else initial_row = intercept;
     if(verbose){
          printf("[DEBUG] remove_ground() --- Original Ground Line Coefficients (m, b): %.2f, %d\r\n", slope, intercept);
          printf("[DEBUG] remove_ground() --- Using Ground Line Coefficients (m, b): %.2f, %d\r\n", slope, upper_intercept);
     }

     double t;
     if(debug_timing) t = (double)cv::getTickCount();

     // For each row in the disparity, mask, and v-map images
     uchar* pix;
     for(int v = initial_row; v < keepMaskRefImg.rows; ++v){
          // Calculate the x-coordinate of the estimated ground line at the current row
          // which will be used as a limit
          int xlim = (int)((float)(v - upper_intercept) / slope);

          // Find all the nonzero pixels in the current row of the v-map
          cv::Mat nonzero;
          cv::Mat refRow = gndLineRefImg.row(v);
          cv::Mat refMask = refRow > 0;
          cv::findNonZero(refMask, nonzero);
          // Find the max and min disparity values that are above the estimated ground line
          int maxx = 0, minx = 1000;
          for(int i = 0; i < nonzero.total(); i++ ){
               // Get the x-coordinate of the current non-zero v-map pixel
               int tmpx = nonzero.at<cv::Point>(i).x;
               // Skip if the current v-map pixel's x-coordinate is "below" the
               // estimated ground-line
               if(tmpx > xlim) continue;
               // Update the current extremes if the current pixels x-coordinate
               // (i.e. the disparity value) is a better option
               if(tmpx > maxx) maxx = tmpx;
               if(tmpx < minx) minx = tmpx;
          }

          // For each pixel in the current column of the disparity and mask images
          pix = keepMaskRefImg.ptr<uchar>(v);
          for(int u = 0; u < keepMaskRefImg.cols; ++u){
               // Fill in the "keep" mask pixel if the disparity value in the
               // corresponding disparity image is in the range of valid pixel values
               // from in the corresponding v-map
               int dvalue = pix[u];
               if( (dvalue >= minx) && (dvalue <= maxx) ) removeMask.at<uchar>(v, u) = 255;
          }
     }
     // Create "keep" mask, use it to filter the original depth image
     cv::Mat keepMask;
     cv::bitwise_not(removeMask, keepMask);
     if(keep_mask) *keep_mask = keepMask.clone();

     // originalDepth.copyTo(filtered_depth, removeMask);
     originalDepth.copyTo(filtered_depth, keepMask);

     if(debug_timing){
          t = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
          printf("[INFO] remove_ground() ---- took %.4lf ms (%.2lf Hz) to filter ground from depth image\r\n", t*1000.0, (1.0/t));
     }

     if(filtered_image) *filtered_image = filtered_depth.clone();
     return 0;
}
int filter_depth_using_ground_line(const cv::Mat& depth, const cv::Mat& disparity,
     const cv::Mat& vmap, std::vector<float> line_params, cv::Mat* filtered_image,
     std::vector<int> line_intercept_offsets, VboatsProcessingImages* image_debugger,
     bool verbose, bool debug_timing)
{
     int err = 0;
     if(image_debugger && image_debugger->visualize_gnd_line_keep_mask){
          cv::Mat debug_image;
          err = filter_depth_using_ground_line(depth, disparity,
               vmap, line_params, filtered_image,
               line_intercept_offsets,
               &debug_image,   // keep_mask visualization
               verbose, debug_timing
          );
          image_debugger->set_gnd_line_filtering_keep_mask(debug_image);
     } else{
          cv::Mat* debug_image = nullptr;
          err = filter_depth_using_ground_line(depth, disparity,
               vmap, line_params, filtered_image,
               line_intercept_offsets,
               debug_image,   // keep_mask visualization
               verbose, debug_timing
          );
     }
     return err;
}

int filter_depth_using_object_candidate_regions(const cv::Mat& depth, const cv::Mat& disparity, const cv::Mat& vmap,
     const vector<vector<cv::Point>>& contours, cv::Mat* filtered_image,
     std::vector<float> line_params, int gnd_line_offset,
     cv::Mat* keep_mask, cv::Mat* vmap_objects,
     std::vector<cv::Rect>* vmap_search_regions,
     bool verbose, bool debug, bool debug_img_info, bool debug_timing)
{
     if(depth.empty()){
          if(verbose) printf("[WARNING] remove_objects() --- Depth Image is empty, skipping object removal.\r\n");
          return -1;
     }
     if(disparity.empty()){
          if(verbose) printf("[WARNING] remove_objects() --- Disparity Image is empty, skipping object removal.\r\n");
          if(filtered_image) *filtered_image = depth.clone();
          return -2;
     }
     if(vmap.empty()){
          if(verbose) printf("[WARNING] remove_objects() --- Vmap Input is empty, skipping object removal.\r\n");
          if(filtered_image) *filtered_image = depth.clone();
          return -3;
     }
     if(contours.empty()){
          if(verbose) printf("[WARNING] remove_objects() --- No object candidate regions provided, skipping object removal.\r\n");
          if(filtered_image) *filtered_image = depth.clone();
          return -4;
     }

     cv::Mat originalDepth, originalDisparity, originalVmap, keepMask;
     originalVmap = vmap.clone();
     originalDepth = depth.clone();
     originalDisparity = disparity.clone();
     cv::Mat vmapMaskRef = cv::Mat::zeros(vmap.rows, vmap.cols, CV_8UC1);
     if(debug_img_info) cvinfo(originalDepth, "remove_objects() --- Depth image at function start: ");

     // Initialize default coefficients in case of ground line estimation failure
     int h = vmap.rows;
     float slope = 0.0;
     int roi_lower_y = h;
     int gnd_line_intercept = h;
     if(!line_params.empty()){
          slope = line_params[0];
          gnd_line_intercept = (int) line_params[1];
     }
     int search_line_intercept = gnd_line_intercept - gnd_line_offset;
     if(debug) printf("[DEBUG] remove_objects() --- Using linear coefficients: slope = %.2f, intercept = %d\r\n", slope, search_line_intercept);

     double t0;
     if(debug_timing) t0 = (double)cv::getTickCount();

     // Initialize storage variables
     int roi_lower_y_edge;        // TODO: Potentially use this for more strict filtering
     vector<int> disparityLims;
     std::vector<cv::Point> contour;
     std::vector<cv::Rect> searchRois;
     for(int i = 0; i < (int) contours.size(); i++){
          contour = contours[i];
          // Extract the XY coordinates (i.e. x coordinate and disparity) for each contour found in the umap
          extract_contour_bounds(contour, nullptr, &disparityLims);
          int dmin = disparityLims.at(0);
          int dmax = disparityLims.at(1);
          float dmid = (float) (dmin + dmax) / 2.0;

          // Find the lower y-coordinate for the current object candidate's
          // (i.e. umap contour) search region
          roi_lower_y = (int)(dmid * slope) + search_line_intercept;
          roi_lower_y_edge = (int)(dmin * slope) + search_line_intercept;

          // Change lower y-coordinate for edge case scenarios
          if(roi_lower_y >= h) roi_lower_y = h;
          else if(roi_lower_y < 0){
               // If no ground line coefficients were used (i.e. unable to
               // determine where the ground is) search the entire height of the
               // v-map, otherwise the "lower" y-coordinate was found outside the
               // limits of the v-map dimensions so set this to zero for skipping
               if(slope != 0){
                    roi_lower_y = 0;
                    roi_lower_y_edge = 0;
               } else roi_lower_y = h;
          }
          if(debug) printf("[DEBUG] remove_objects() ------ Contour[%d]: dmin, dmid, dmax = (%d, %d, %d) |  ROI Rect = (%d, 0) -> (%d, %d)\r\n", i, dmin, (int) dmid, dmax, dmin, dmax,roi_lower_y);
          if(roi_lower_y == 0) continue;

          // Extract the pixels in the v-map contained in the ROI corresponding
          // to the current object candidate
          cv::Rect roiRect = cv::Rect( cv::Point(dmin, 0), cv::Point(dmax, roi_lower_y) );
          if(vmap_search_regions) searchRois.push_back(roiRect);
          cv::Mat roi = originalVmap(roiRect);

          // Copy pixels found in the ROI of the v-map into the blank v-map mask
          roi.copyTo(vmapMaskRef(roiRect));
          if(debug_img_info) cvinfo(roi, "remove_objects() --- ROI added to vMask: ");
     }
     if(vmap_objects) *vmap_objects = vmapMaskRef.clone();

     double t0_masking_start;
     if(debug_timing) t0_masking_start = (double)cv::getTickCount();

     // Create a ForEach operator object used for converting a disparity image
     // into a binary "keep" mask using a vmap with nonzero pixel values where
     // object candidates (i.e. contours found in corresponding umap) may reside
     ForEachObsMaskGenerator masker(vmapMaskRef, h, 256);

     // Convert the original disparity image into a "keep" mask, and store it  as a seperate variable for code readability
     if(debug_img_info) cvinfo(originalDisparity, "remove_objects() --- originalDisparity before forEach: ");
     originalDisparity.forEach<uchar>(masker);
     keepMask = originalDisparity.clone();
     if(debug_img_info) cvinfo(originalDisparity, "remove_objects() --- originalDisparity after forEach: ");
     masker.remove(); // ForEach operator Cleanup

     if(debug_timing){
          t0_masking_start = ((double)cv::getTickCount() - t0_masking_start)/cv::getTickFrequency();
          printf("[INFO] remove_objects() ---- took %.4lf ms (%.2lf Hz) to create \"keep\" mask from disparity image using ForEach operator\r\n", t0_masking_start*1000.0, (1.0/t0_masking_start));
     }

     // Filter original depth image using the "keep" mask created from the object filtered disparity image
     cv::Mat filteredDepth;
     if(!keepMask.empty()){
          originalDepth.copyTo(filteredDepth, keepMask);
          if(keep_mask) *keep_mask = keepMask.clone();
     } else{
          filteredDepth = originalDepth;
          if(verbose){ printf("[WARNING] remove_objects() --- \"keep\" mask created using obstacle candidate is somehow empty. Resulting depth image hasn\'t been filtered.\r\n"); }
     }

     if(debug_timing){
          t0 = ((double)cv::getTickCount() - t0)/cv::getTickFrequency();
          printf("[INFO] remove_objects() ---- took %.4lf ms (%.2lf Hz) to filter depth image using object candidate regions\r\n", t0*1000.0, (1.0/t0));
     }

     if(filtered_image) *filtered_image = filteredDepth.clone();
     if(vmap_search_regions) *vmap_search_regions = std::vector<cv::Rect>{searchRois.begin(), searchRois.end()};
     return 0;
}
int filter_depth_using_object_candidate_regions(const cv::Mat& depth, const cv::Mat& disparity, const cv::Mat& vmap,
     const vector<vector<cv::Point>>& contours, cv::Mat* filtered_image,
     std::vector<float> line_params, int gnd_line_offset,
     VboatsProcessingImages* image_debugger,
     bool verbose, bool debug, bool debug_img_info, bool debug_timing)
{
     int err = 0;
     if(image_debugger){
          cv::Mat* debug_image = nullptr;
          cv::Mat* debug_image2 = nullptr;
          if(image_debugger->visualize_obj_candidate_keep_mask) debug_image = new cv::Mat();
          if(image_debugger->visualize_vmap_candidates_img) debug_image2 = new cv::Mat();
          err = filter_depth_using_object_candidate_regions(depth, disparity,
               vmap, contours, filtered_image, line_params, gnd_line_offset,
               debug_image,  // keep_mask visualization
               debug_image2,  // vmap_objects visualization
               nullptr,  // vmap_search_regions visualization
               verbose, debug, debug_img_info, debug_timing
          );
          if(image_debugger->visualize_obj_candidate_keep_mask){
               image_debugger->set_vmap_object_candidates_image(*debug_image);
               delete debug_image;
          }
          if(image_debugger->visualize_vmap_candidates_img){
               image_debugger->set_obj_candidate_filtering_keep_mask(*debug_image2);
               delete debug_image2;
          }
     } else{
          err = filter_depth_using_object_candidate_regions(depth, disparity,
               vmap, contours, filtered_image, line_params, gnd_line_offset,
               nullptr,  // keep_mask visualization
               nullptr,  // vmap_objects visualization
               nullptr,  // vmap_search_regions visualization
               verbose, debug, debug_img_info, debug_timing
          );
     }
     return err;
}
