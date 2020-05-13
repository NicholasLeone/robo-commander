#ifndef UTILITIES_IMAGE_UTILS_H_
#define UTILITIES_IMAGE_UTILS_H_

#include <string.h>
#include <vector>

#include <opencv2/opencv.hpp>

using namespace std;

/** Basic Functions */
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

/** Visualization Functions */
cv::Mat imCvtCmap(const cv::Mat& img);
int imshowCmap(const cv::Mat& img, std::string title);

/** ForEach Functors */
template<typename dtype> struct ForEachDepthConverter{
     dtype m_gain;
     ForEachDepthConverter(dtype gain){
          m_gain = gain;
     }
     void operator()(dtype& pixel, const int * idx) const{
          if(!std::isfinite(pixel)) pixel = 0;
          else if(pixel != 0.0){ pixel = m_gain / pixel; }
     }
};

/** Used by camera D4xx class */
template<typename Pixel> struct ForEachOperator{
     Pixel m_gain;
     ForEachOperator(Pixel gain){ m_gain = gain; }
     void operator()(Pixel& pixel, const int * idx) const {
          if(pixel != 0.0){ pixel = m_gain / pixel;}
     }
};

// template<typename dtype> struct ForEachDepthConverter{
//      const dtype m_gain;
//      const dtype m_min;
//      const dtype m_max;
//      ForEachDepthConverter(dtype gain) : m_gain(gain), m_min(0), m_max(0){}
//      ForEachDepthConverter(dtype gain, dtype min, dtype max) : m_gain(gain), m_min(min), m_max(max){}
//      void operator()(dtype& pixel, const int * idx) const{
//           if(!std::isfinite(pixel)) pixel = 0;
//           else if(pixel != 0){
//                dtype tmpVal = pixel;
//                if((m_min != 0) && (tmpVal < m_min)) tmpVal = m_min;
//                else if((m_max != 0) && (tmpVal > m_max)) tmpVal = m_max;
//                pixel = ((dtype) m_gain / tmpVal);
//           }
//      }
// };

template<typename dtype> struct ForEachPrepareDepthConverter{
     const dtype m_min;
     const dtype m_max;
     ForEachPrepareDepthConverter() : m_min(0), m_max(0){}
     ForEachPrepareDepthConverter(dtype abs_min, dtype abs_max) : m_min(abs_min), m_max(abs_max){}
     void operator()(dtype& pixel, const int * idx) const{
          if(!std::isfinite(pixel)){ pixel = 0; }
          else if(pixel != 0){
               if(m_min != 0){ if(pixel < m_min) pixel = m_min; }
               if(m_max != 0){ if(pixel > m_max) pixel = m_max; }
          }
     }
};

/** Templated structure for fast generation of object segmentation mask */
struct ForEachObsMaskGenerator{
     int ** m_table;
     const size_t m_rows;
     const size_t m_cols;
     ForEachObsMaskGenerator(const cv::Mat& input_mask, int num_rows, int num_cols) : m_rows(num_rows), m_cols(num_cols){
          cv::Mat nonzero;
          m_table = new int*[num_rows]();
          for(int v = 0; v < num_rows; v++){
               if(m_table[v]) delete[] m_table[v];
               m_table[v] = new int[num_cols]();
               memset(m_table[v], 0, num_cols*sizeof(int));
               cv::Mat refRow = input_mask.row(v);
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

/** TODO */
// def histogram_sliding_filter(hist, window_size=16, flag_plot=False):


#endif // UTILITIES_IMAGE_UTILS_H_
