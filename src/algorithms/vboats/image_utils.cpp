#include <iostream>
#include <stdio.h>
#include "algorithms/vboats/image_utils.h"

using namespace std;

std::string cvtype2str(int type){
     std::string r;
     uchar depth = type & CV_MAT_DEPTH_MASK;
     uchar chans = 1 + (type >> CV_CN_SHIFT);

     switch ( depth ) {
          case CV_8U:  r = "8U"; break;
          case CV_8S:  r = "8S"; break;
          case CV_16U: r = "16U"; break;
          case CV_16S: r = "16S"; break;
          case CV_32S: r = "32S"; break;
          case CV_32F: r = "32F"; break;
          case CV_64F: r = "64F"; break;
          default:     r = "User"; break;
     }
     r += "C";
     r += (chans+'0');
     return r;
}
std::string cvStrSize(const char* name, const cv::Mat& mat){
     std::string str = format("cv::Size \'%s\' [%d, %d, %d, %s]", name, mat.cols, mat.rows, mat.channels(), cvtype2str(mat.type()).c_str());
     return str;
}

void spread_image(const cv::Mat& input, cv::Mat* output, const cv::Size& target_size, double* fx, double* fy, bool verbose){
     cv::Mat _output;
     double _fx = (double)(target_size.width/((double) input.cols));
     double _fy = (double)(target_size.height/((double) input.rows));
     cv::resize(input, _output, target_size, 0, 0, cv::INTER_LINEAR);
     if(verbose)
          printf("[INFO] spread_image() --- Input Img (%d, %d) w/ Target Size (%d, %d) --> New Img (%d, %d) using factors (%.2f, %.2f)\r\n",
               input.cols, input.rows, target_size.width, target_size.height, _output.cols, _output.rows, _fx, _fy);
     if(fx) *fx = _fx;
     if(fy) *fy = _fy;
     *output = _output;
}
void spread_image(const cv::Mat& input, cv::Mat* output, double fx, double fy, cv::Size* new_size, bool verbose){
     cv::Mat _output;
     cv::resize(input, _output, cv::Size(), fx, fy, cv::INTER_LINEAR);
     cv::Size _new_size = cv::Size(_output.cols, _output.rows);
     if(verbose)
          printf("[INFO] spread_image() --- Input Img (%d, %d) w/ factors (%.2f, %.2f) --> New Img (%d, %d) using cv::Size(%d, %d)\r\n",
               input.cols, input.rows, fx, fy, _output.cols, _output.rows, _new_size.width, _new_size.height);
     if(new_size) *new_size = _new_size;
     *output = _output;
}

void unspread_image(const cv::Mat& input, cv::Mat* output, const cv::Size& target_size, float* fx, float* fy, bool verbose){
     cv::Mat _output;
     double _fx = (double)(target_size.width/((double) input.cols));
     double _fy = (double)(target_size.height/((double) input.rows));
     cv::resize(input, _output, target_size, 0, 0, cv::INTER_AREA);
     if(verbose)
          printf("[INFO] unspread_image() --- Input Img (%d, %d) w/ Target Size (%d, %d) --> New Img (%d, %d) using factors (%.2f, %.2f)\r\n",
               input.cols, input.rows, target_size.width, target_size.height, _output.cols, _output.rows, _fx, _fy);
     if(fx) *fx = _fx;
     if(fy) *fy = _fy;
     *output = _output;
}
void unspread_image(const cv::Mat& input, cv::Mat* output, float fx, float fy, cv::Size* new_size, bool verbose){
     cv::Mat _output;
     cv::resize(input, _output, cv::Size(), fx, fy, cv::INTER_AREA);
     cv::Size _new_size = cv::Size(_output.cols, _output.rows);
     if(verbose)
          printf("[INFO] spread_image() --- Input Img (%d, %d) w/ factors (%.2f, %.2f) --> New Img (%d, %d) using cv::Size(%d, %d)\r\n",
               input.cols, input.rows, fx, fy, _output.cols, _output.rows, _new_size.width, _new_size.height);
     if(new_size) *new_size = _new_size;
     *output = _output;
}

int strip_image(const cv::Mat& input, vector<cv::Mat>* strips, int nstrips, bool cut_horizontally, bool visualize, bool verbose){
     int i = 0;
     int ncols, nrows;
     int h = input.rows;
     int w = input.cols;
     std::string strDebug;
     if(cut_horizontally){
          ncols = nstrips;
          nrows = 1;
          strDebug = "horizontally";
     } else{
          ncols = 1;
          nrows = nstrips;
          strDebug = "vertically";
     }
     int dx = w / ncols;
     int dy = h / nrows;
     if( (w % ncols == 0) && (h % nrows == 0) ){
          printf("[INFO] strip_image() ---- Stripping input image (%d, %d) %s into %d strips of size %d, %d.\r\n",h, w,strDebug.c_str(), nstrips, dx,dy);
     }else if(w % ncols != 0){
          printf("[ERROR] strip_image() ---- Please choose another value for nstrips.\r\n");
          return -1;
     }else if(h % nrows != 0){
          printf("[ERROR] strip_image() ---- Please choose another value for nstrips.\r\n");
          return -1;
     }

     cv::Mat display;
     vector<cv::Mat> _strips;
     if(visualize) display = input.clone();
     for(int x = 0; x < w; x += dx){
          for(int y = 0; y < h; y += dy){
               if(verbose) printf("[INFO] strip_image() ---- Current location (x,y): (%d, %d)\r\n",x,y);
               _strips.push_back(input(cv::Rect(x, y, dx, dy)).clone());
               i++;
               if(visualize){
                    cv::rectangle(display, cv::Point(x, y), cv::Point(x + (display.cols / ncols) - 1, y + (display.rows / nrows) - 1), CV_RGB(255, 0, 0), 1);
                    cv::imshow("mask", display);
                    cv::waitKey(0);
               }
          }
     }
     *strips = _strips;
     return 0;
}
int merge_strips(const vector<cv::Mat>& strips, cv::Mat* merged, bool merge_horizontally, bool visualize, bool verbose){
     cv::Mat _img;
     if(merge_horizontally) cv::hconcat(strips, _img);
     else cv::vconcat(strips, _img);

     if(verbose) printf("[INFO] merge_strips() ---- Reconstructed image (%d, %d).\r\n",_img.rows, _img.cols);
     if(visualize){
          cv::imshow("reconstructed image", _img);
          cv::waitKey(0);
     }
     *merged = _img;
     return 0;
}


/** TODO */
// def histogram_sliding_filter(hist, window_size=16, flag_plot=False):
// 	avg_hist = np.zeros_like(hist).astype(np.int32)
// 	sliding_window = np.ones((window_size,))/window_size
//
// 	try:
// 		n, depth = hist.shape
// 	except:
// 		n = hist.shape
// 		depth = None
//
// 	if flag_plot == True:
// 		plt.figure(1)
// 		plt.clf()
// 		plt.title('Smoothed Histogram of the image')
//
// 	if depth == None:
// 		avg_hist = np.convolve(hist[:], sliding_window , mode='same')
// 		if flag_plot == True:
// 			plt.plot(range(avg_hist.shape[0]), avg_hist[:])
// 	else:
// 		for channel in range(depth):
// 			tmp_hist = np.convolve(hist[:,channel], sliding_window , mode='same')
// 			avg_hist[:,channel] = tmp_hist
// 			if flag_plot == True:
// 				plt.plot(range(avg_hist.shape[0]), avg_hist[:,channel])
// 				# plt.plot(range(avg_hist.shape[0]), avg_hist[:,1])
// 				# plt.plot(range(avg_hist.shape[0]), avg_hist[:,2])
// 	return avg_hist
