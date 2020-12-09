#ifndef ROBOCOMMANDER_UTILITIES_IMAGE_UTILS_H_
#define ROBOCOMMANDER_UTILITIES_IMAGE_UTILS_H_

#include <vector>
#include <string.h>

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/version.hpp> /* For OpenCV backwards compatibility */

#ifdef WITH_CUDA
#include <opencv2/gpu.hpp>
#include <opencv2/core/cuda.hpp>
#endif

using namespace std;

std::string cvtype2str(int type);
std::string cvtype2str(cv::Mat mat);
std::string cvStrSize(const char* name, const cv::Mat& mat);
void cvinfo(const cv::Mat& mat, const char* label);
void print_cvmat(std::string header, const cv::Mat& mat, std::string rsep = "  ");

/** Image Maniuplation Functions */
int strip_image(const cv::Mat& input, vector<cv::Mat>* strips, int nstrips = 5,
     bool cut_horizontally = true, bool visualize=false, bool verbose=false
);
int merge_strips(const vector<cv::Mat>& strips, cv::Mat* merged,
     bool merge_horizontally = true, bool visualize=false, bool verbose=false
);
void spread_image(const cv::Mat& input, cv::Mat* output, const cv::Size& target_size,
     double* fx, double* fy, bool verbose = false
);
void spread_image(const cv::Mat& input, cv::Mat* output, double fx, double fy,
     cv::Size* new_size, bool verbose = false
);
void unspread_image(const cv::Mat& input, cv::Mat* output, const cv::Size& target_size,
     double* fx, double* fy, bool verbose = false
);
void unspread_image(const cv::Mat& input, cv::Mat* output, double fx, double fy,
     cv::Size* new_size, bool verbose = false
);

cv::Mat rotate_image(const cv::Mat& input, double angle = 0.0);

#ifdef WITH_CUDA
// cv::Mat rotate_image(const cv::cuda::GpuMat& input, double angle = 0.0);
#endif

/** Visualization and plotting utility functions */
cv::Mat imCvtCmap(const cv::Mat& img);
int imshowCmap(const cv::Mat& img, std::string title);
int pplot(cv::Mat image, std::string title, bool blocking = false);
int pplots(std::vector<cv::Mat> images, long cols, long rows, std::string title, bool blocking = false);

/** Encoding / Decoding utility functions */
std::string img_to_str_simple(cv::Mat image, std::string encoding = "jpg");
std::string img_to_str(cv::Mat image, double* min = nullptr, double* max = nullptr, float* scale = nullptr, bool preprocess = true, std::string encoding = "jpg");

/** Templated struct for fast conversion of a depth image into disparity using naive method */
template<typename dtype> struct ForEachNaiveDepthConverter{
     dtype m_gain;
     ForEachNaiveDepthConverter(dtype gain){ m_gain = gain; }
     void operator()(dtype& pixel, const int * idx) const {
          if((std::isfinite(pixel)) && (pixel != 0.0)){ pixel = m_gain / pixel; }
     }
};
/** Templated struct for fast conversion of depth image into disparity */
template<typename dtype> struct ForEachPrepareDepthConverter{
     const dtype m_min;
     const dtype m_max;
     ForEachPrepareDepthConverter() : m_min(0), m_max(0){}
     ForEachPrepareDepthConverter(dtype abs_min, dtype abs_max) : m_min(abs_min), m_max(abs_max){}
     void operator()(dtype& pixel, const int * idx) const{
          if(!std::isfinite(pixel)){ pixel = 0; }
          else if(pixel != 0){
               if(m_min != 0){ if(pixel < m_min) pixel = 0; }
               if(m_max != 0){ if(pixel > m_max) pixel = 0; }
          }
     }
};
template<typename dtype> struct ForEachDepthConverter{
     const dtype m_gain;
     const dtype m_min;
     const dtype m_max;
     ForEachDepthConverter(dtype gain) : m_gain(gain), m_min(0), m_max(0){}
     ForEachDepthConverter(dtype gain, dtype min, dtype max) : m_gain(gain), m_min(min), m_max(max){}
     void operator()(dtype& pixel, const int * idx) const{
          if(!std::isfinite(pixel)) pixel = 0;
          else if(pixel != 0){
               dtype tmpVal = pixel;
               if((m_min != 0) && (tmpVal < m_min)) tmpVal = m_min;
               else if((m_max != 0) && (tmpVal > m_max)) tmpVal = m_max;
               pixel = ((dtype) m_gain / tmpVal);
          }
     }
};
template<typename dtype> struct ForEachSaturateDepthLimits{
     const dtype m_gain;
     const dtype m_min;
     const dtype m_max;
     const dtype m_fx;
     const dtype m_fy;
     const dtype m_px;
     const dtype m_py;
     const dtype x_min;
     const dtype x_max;
     const dtype y_min;
     const dtype y_max;
     ForEachSaturateDepthLimits() : m_gain(0.0), m_min(0.0), m_max(0.0), m_fx(0.0), m_fy(0.0),
          m_px(0.0), m_py(0.0), x_min(0.0), x_max(0.0), y_min(0.0), y_max(0.0){}
     ForEachSaturateDepthLimits(dtype gain) :
          m_gain(gain), m_min(0.0), m_max(0.0), m_fx(0.0), m_fy(0.0),
          m_px(0.0), m_py(0.0), x_min(0.0), x_max(0.0), y_min(0.0), y_max(0.0){}
     ForEachSaturateDepthLimits(dtype gain, dtype abs_min, dtype abs_max) :
          m_gain(gain), m_min(abs_min), m_max(abs_max), m_fx(0.0), m_fy(0.0),
          m_px(0.0), m_py(0.0), x_min(0.0), x_max(0.0), y_min(0.0), y_max(0.0){}
     ForEachSaturateDepthLimits(dtype gain, dtype abs_min, dtype abs_max,
          dtype fx, dtype fy, dtype px, dtype py,
          dtype minX, dtype maxX, dtype minY, dtype maxY) : m_gain(gain),
          m_min(abs_min), m_max(abs_max), m_fx(fx), m_fy(fy), m_px(px), m_py(py),
          x_min(minX), x_max(maxX), y_min(minY), y_max(maxY){}
     void operator()(dtype& pixel, const int * idx) const{
          if(!std::isfinite(pixel)){ pixel = 0; }
          else if(pixel != 0){
               dtype depth = pixel;
               // Saturate pixel value based on geometric limits -- X DIMENSION
               // Note: If supplied and camera intrinsics are known
               bool clip_x = false;
               if(m_fx != 0){
                    float xNum = ((float) idx[1] - (float) m_px) * (float) depth;
                    float xLoc = (float) xNum / (float) m_fx;
                    if( (x_min != 0) && ((dtype) xLoc < x_min) ){ clip_x = true; }
                    if( (x_max != 0) && ((dtype) xLoc > x_max) ){ clip_x = true; }
               }
               // Saturate pixel value based on geometric limits -- Y DIMENSION
               // Note: If supplied and camera intrinsics are known
               bool clip_y = false;
               if(m_fy != 0){
                    float yNum = ((float) idx[0] - (float) m_py) * (float) depth;
                    float yLoc = (float) yNum / (float) m_fy;
                    if( (y_min != 0) && ((dtype) yLoc < y_min) ){ clip_y = true; }
                    if( (y_max != 0) && ((dtype) yLoc > y_max) ){ clip_y = true; }
               }

               // Finally convert current pixel's depth value to its
               // corresponding disparity value, unless we are clipping this pixel
               dtype tmpVal;
               if(clip_x || clip_y){ tmpVal = 0; }
               else{
                    dtype tmpDepth = depth;
                    // Saturate pixel value to depth limits (IF SUPPLIED)
                    if( (m_min != 0) && (depth < m_min) ){ tmpDepth = m_min; }
                    else if( (m_max != 0) && (depth > m_max) ){ tmpDepth = m_max; }
                    tmpVal = (dtype) m_gain / tmpDepth;
               }
               // Actually set pixel value to new value in original input image
               pixel = tmpVal;
          }
     }
};
/** Templated structure for fast generation of object segmentation mask.
* Creates a binary LUT using nonzero pixel values in the input vmap
*/
struct ForEachObsMaskGenerator{
     int ** m_table;
     const size_t m_rows;
     const size_t m_cols;
     ForEachObsMaskGenerator(const cv::Mat& vmap_reference, int num_rows, int num_cols) : m_rows(num_rows), m_cols(num_cols){
          cv::Mat nonzero;
          m_table = new int*[num_rows]();
          for(int v = 0; v < num_rows; v++){
               if(m_table[v]) delete[] m_table[v];
               m_table[v] = new int[num_cols]();
               memset(m_table[v], 0, num_cols*sizeof(int));
               cv::Mat refRow = vmap_reference.row(v);
               cv::findNonZero(refRow, nonzero);
               for(int u = 0; u < nonzero.total(); ++u){
                    int tmpx = nonzero.at<cv::Point>(u).x;
                    m_table[v][tmpx] = 255;
               }
          }
     }
     void remove(){
          for(auto i = 0; i < m_rows; i++){
               if(m_table[i]) delete[] m_table[i];
               m_table[i] = nullptr;
          }
          if(m_table) delete[] m_table;
          m_table = nullptr;
     }

     void operator()(uchar& pixel, const int * position) const {
          if((position[0] < m_rows) && (pixel < m_cols)){ pixel = (uchar) m_table[position[0]][pixel]; }
     }
};

#endif // ROBOCOMMANDER_UTILITIES_IMAGE_UTILS_H_
