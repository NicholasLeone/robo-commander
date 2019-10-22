#include "algorithms/vboats/image_utils.h"

using namespace cv;

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
