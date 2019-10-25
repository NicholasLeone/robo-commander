#include <iostream>
#include <stdio.h>
#include <math.h>       /* ceil, cos, sin */

#include "base/definitions.h"
#include "algorithms/vboats/image_utils.h"

using namespace std;

std::string cvtype2str(int type){
     std::string r;
     uchar depth = type & CV_MAT_DEPTH_MASK;
     uchar chans = 1 + (type >> CV_CN_SHIFT);

     switch(depth){
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
std::string cvtype2str(cv::Mat mat){
     int type = mat.type();
     std::string r;
     uchar depth = type & CV_MAT_DEPTH_MASK;
     uchar chans = 1 + (type >> CV_CN_SHIFT);

     switch(depth){
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
     std::string str = format("\'%s\' [%d, %d, %d, %s]", name, mat.cols, mat.rows, mat.channels(), cvtype2str(mat.type()).c_str());
     return str;
}
void cvinfo(const cv::Mat& mat, const char* label){
     if(!label) printf("%s\r\n",cvStrSize("Matrix",mat).c_str());
     else printf("%s\r\n",cvStrSize(label,mat).c_str());
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
          if(verbose) printf("[INFO] strip_image() ---- Stripping input image (%d, %d) %s into %d strips of size %d, %d.\r\n",h, w,strDebug.c_str(), nstrips, dx,dy);
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

void filter_disparity_vmap(const cv::Mat& input, cv::Mat* output, vector<float>* thresholds, bool verbose, bool visualize){
     if(verbose) printf("%s\r\n",cvStrSize("Vmap Filtering Input",input).c_str());
     vector<float> threshs;
     vector<float> default_threshs = {0.15,0.15,0.01,0.01};
     if(!thresholds) threshs = default_threshs;
     else threshs = *thresholds;

     int err;
     cv::Mat filtered;
     cv::Mat imgCopy = input.clone();
     int nThreshs = threshs.size();

     int h = imgCopy.rows;
     int w = imgCopy.cols;
     int c = imgCopy.channels();
     if(visualize){
          cv::namedWindow("input", cv::WINDOW_NORMAL );
          cv::imshow("input", imgCopy);
          cv::waitKey(0);
     }

     double minVal, maxVal;
     cv::minMaxLoc(imgCopy, &minVal, &maxVal);

     int nStrips = int(ceil(maxVal/255.0) * nThreshs);
     int vMax = (int)maxVal;
     if(verbose) printf("vMax, nThreshs: %d, %d\r\n", vMax, nStrips);

     int divSection = 3;
     int dSection = int(h/float(divSection));

     cv::Rect topHalfRoi = cv::Rect(0,0,w,dSection);
     cv::Rect botHalfRoi = cv::Rect(0,dSection,w,h-dSection);
     if(verbose){
          std::cout << "Top Portion ROI: " << topHalfRoi << std::endl;
          std::cout << "Bottom Portion ROI: " << botHalfRoi << std::endl;
     }

     cv::Mat topHalf = imgCopy(topHalfRoi);
     cv::Mat botHalf = imgCopy(botHalfRoi);
     if(visualize){
          // cv::namedWindow("top portion", cv::WINDOW_NORMAL );
          // cv::namedWindow("bottom portion", cv::WINDOW_NORMAL );
          // cv::imshow("top portion", topHalf);
          // cv::imshow("bottom portion", botHalf);
          // cv::waitKey(0);
     }

     cv::minMaxLoc(topHalf, &minVal, &maxVal);
     int tmpThresh = int(maxVal*0.65);
     if(verbose) printf("tops max, tmpThresh: %d, %d\r\n", int(maxVal), tmpThresh);

     cv::Mat topThreshed, prefiltered;
     threshold(topHalf, topThreshed, tmpThresh, 255, cv::THRESH_TOZERO);
     // threshold(topHalf, topThreshed, tmpThresh, 255, cv::THRESH_BINARY);
     topThreshed.copyTo(imgCopy(topHalfRoi));
     if(visualize){
          // cv::namedWindow("thresholded portion", cv::WINDOW_NORMAL );
          // cv::namedWindow("thresholded portion", cv::WINDOW_NORMAL );
          // cv::imshow("thresholded portion", topThreshed);
          // cv::imshow("input image w/ thresholded portion", imgCopy);
          // cv::waitKey(0);
     }

     int idx = 0;
     cv::Scalar cvMean, cvStddev;
     std::vector<cv::Mat> strips;
     std::vector<cv::Mat> pStrips;
     err = strip_image(imgCopy, &strips, nStrips, true);
     for(cv::Mat strip : strips){
          cv::minMaxLoc(strip, &minVal, &maxVal);
          cv::meanStdDev(strip, cvMean, cvStddev);
          int tmpMax = (int) maxVal;
          double tmpMean = (double) cvMean[0];
          double tmpStd = (double) cvStddev[0];
          if(tmpMean == 0.0){
               pStrips.push_back(strip.clone());
               idx++;
               continue;
          }

          if(verbose) printf(" ---------- [Thresholding Strip %d] ---------- \r\n\tMax = %.1f, Mean = %.1f, Std = %.1f\r\n", idx,tmpMax,tmpMean,tmpStd);
          double maxValRatio = (double) vMax/255.0;
          double relRatio = (double)(tmpMax-tmpStd)/(double)vMax;
          double relRatio2 = (tmpMean)/(double)(tmpMax);
          if(verbose) printf("\tRatios: %.3f, %.3f, %.3f\r\n", maxValRatio, relRatio, relRatio2);

          double tmpGain;
          if(relRatio >= 0.4) tmpGain = relRatio + relRatio2;
          else tmpGain = 1.0 - (relRatio + relRatio2);

          int thresh = int(threshs[idx] * tmpGain * tmpMax);
          if(verbose) printf("\tGain = %.2f, Thresh = %d\r\n", tmpGain,thresh);

          cv::Mat tmpStrip;
          threshold(strip, tmpStrip, thresh, 255,cv::THRESH_TOZERO);
          pStrips.push_back(tmpStrip.clone());
          if(visualize){
               // cv::imshow("strip", strip);
               // cv::imshow("thresholded strip", tmpStrip);
               // cv::waitKey(0);
          }
          idx++;
     }

     err = merge_strips(pStrips, &filtered);
     if(visualize){
          cv::namedWindow("reconstructed thresholded input", cv::WINDOW_NORMAL );
          cv::imshow("reconstructed thresholded input", filtered);
          cv::waitKey(0);
     }
     if(output) *output = filtered;
}
void filter_disparity_umap(const cv::Mat& input, cv::Mat* output, vector<float>* thresholds, bool verbose, bool visualize){
     if(verbose) printf("%s\r\n",cvStrSize("Umap Filtering Input",input).c_str());
     vector<float> threshs;
     vector<float> default_threshs = {0.25,0.15,0.35,0.35};
     if(!thresholds) threshs = default_threshs;
     else threshs = *thresholds;

     int err;
     cv::Mat filtered;
     cv::Mat imgCopy = input.clone();
     int nThreshs = threshs.size();

     int h = imgCopy.rows;
     int w = imgCopy.cols;
     int c = imgCopy.channels();
     if(visualize){
          cv::namedWindow("input", cv::WINDOW_NORMAL );
          cv::imshow("input", imgCopy);
          cv::waitKey(0);
     }

     double minVal, maxVal;
     cv::minMaxLoc(imgCopy, &minVal, &maxVal);

     int uMax = (int)maxVal;
     int nStrips = int(ceil(maxVal/255.0) * nThreshs);
     if(verbose) printf("uMax, nThreshs: %d, %d\r\n", uMax, nStrips);

     threshold(imgCopy, imgCopy, 2, 255, cv::THRESH_TOZERO);

     int idx = 0;
     cv::Scalar cvMean, cvStddev;
     std::vector<cv::Mat> strips;
     std::vector<cv::Mat> pStrips;
     err = strip_image(imgCopy, &strips, nStrips, false);
     for(cv::Mat strip : strips){
          cv::minMaxLoc(strip, &minVal, &maxVal);
          cv::meanStdDev(strip, cvMean, cvStddev);
          int tmpMax = (int) maxVal;
          double tmpMean = (double) cvMean[0];
          double tmpStd = (double) cvStddev[0];
          if(tmpMean == 0.0){
               pStrips.push_back(strip.clone());
               idx++;
               continue;
          }

          if(verbose) printf(" ---------- [Thresholding Strip %d] ---------- \r\n\tMax = %.1f, Mean = %.1f, Std = %.1f\r\n", idx,tmpMax,tmpMean,tmpStd);
          double maxValRatio = (double) uMax/255.0;
          double relRatio = (double)(tmpMax-tmpStd)/(double)uMax;
          double relRatio2 = (tmpMean)/(double)(tmpMax);
          if(verbose) printf("\tRatios: %.3f, %.3f, %.3f\r\n", maxValRatio, relRatio, relRatio2);

          double tmpGain;
          if(relRatio >= 0.4) tmpGain = relRatio + relRatio2;
          else tmpGain = 1.0 - (relRatio + relRatio2);

          int thresh = int(threshs[idx] * tmpGain * tmpMax);
          if(verbose) printf("\tGain = %.2f, Thresh = %d\r\n", tmpGain,thresh);

          cv::Mat tmpStrip;
          threshold(strip, tmpStrip, thresh, 255,cv::THRESH_TOZERO);
          pStrips.push_back(tmpStrip.clone());
          if(visualize){
               // cv::imshow("strip", strip);
               // cv::imshow("thresholded strip", tmpStrip);
               // cv::waitKey(0);
          }
          idx++;
     }

     err = merge_strips(pStrips, &filtered, false);
     if(visualize){
          cv::namedWindow("reconstructed thresholded input", cv::WINDOW_NORMAL );
          cv::imshow("reconstructed thresholded input", filtered);
          cv::waitKey(0);
     }

     if(output) *output = filtered;
}

int find_ground_lines(const cv::Mat& vmap, cv::Mat* found_lines, int hough_thresh, bool verbose){
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
     // std::cout << "test: " << out[0] << std::endl;

     if(rhos) *rhos = out[0];
     if(thetas) *thetas = out[1];
     return n;
}
int find_ground_lines(const cv::Mat& vmap, vector<cv::Vec2f>* found_lines, int hough_thresh, bool verbose){
     vector<cv::Vec2f> _lines;
     cv::HoughLines(vmap, _lines, 1, CV_PI/180, hough_thresh);
     int n = _lines.size();
     if(verbose) printf("[INFO] find_ground_lines() ---- Found %d hough lines in image.\r\n", n);
     if(found_lines) *found_lines = _lines;
     return n;
}

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

int estimate_ground_line(const vector<cv::Vec2f>& lines, float* best_slope,
     int* best_intercept, double gnd_deadzone, double minDeg, double maxDeg,
     bool verbose, bool debug_timing)
{
     double t;
     if(debug_timing) t = (double)cv::getTickCount();

     float dAng = gnd_deadzone * M_DEG2RAD;
     float minAng = (minDeg * M_DEG2RAD) + CV_PI;
     float maxAng = (maxDeg * M_DEG2RAD) + CV_PI;

     vector<cv::Vec2f> buf;
     int nLines = lines.size();
     if(nLines <= 0) return -1;

     float sumRad = 0.0;
     float tmpRho, tmpAng;
     for(int i = 0; i < nLines; i++){
          tmpRho = lines[i][0];
          tmpAng = lines[i][1];
          if(tmpAng >= minAng && tmpAng <= maxAng){
               sumRad += tmpAng;
               buf.push_back(cv::Vec2f(tmpRho, tmpAng));
          }
     }

     int num = 0;
     int tmpB = 0, bestB = 0;
     float tmpM = 0.0, bestM = 0.0;
     float avgAng = sumRad / float(buf.size());
     float minAvg = avgAng - dAng;
     float maxAvg = avgAng + dAng;
     // printf("[INFO] %d filtered Lines --- avg angle = %.2f deg\r\n", buf.size(), avgAng*M_RAD2DEG);
     for(cv::Vec2f tmpLine : buf){
          if(tmpLine[1] >= minAvg && tmpLine[1] <= maxAvg){
               get_hough_line_params(tmpLine[0],tmpLine[1], &tmpM, &tmpB);
               if(tmpB > bestB){ bestM = tmpM; bestB = tmpB; }
               num++;
          }
     }
     if(verbose) printf("[INFO] estimate_ground_line() ---- estimated ground line (m = %.2f, b = %d) found from %d / %d hough lines\r\n", bestM, bestB, num,nLines);
     if(debug_timing){
          t = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
          printf("[INFO] estimate_ground_line() ---- took %.4lf ms (%.2lf Hz)\r\n", t*1000.0, (1.0/t));
     }
     if(best_slope) *best_slope = bestM;
     if(best_intercept) *best_intercept = bestB;
     return num;
}

bool is_ground_present(const cv::Mat& vmap, float* best_slope, int* best_intercept,
     int hough_thresh, double gnd_deadzone, double minDeg, double maxDeg, bool verbose, bool debug_timing, bool visualize)
{
     cv::Mat rs, angs;
     std::vector<cv::Vec2f> lines;

     double t;
     if(debug_timing) t = (double)cv::getTickCount();
     // int nLines = find_ground_lines(vmap, &rs, &angs, hough_thresh);
     int nLines = find_ground_lines(vmap, &lines, hough_thresh);
     if(nLines <= 0) return false;

     float m;  int b;
     int nUsed = estimate_ground_line(lines, &m, &b, gnd_deadzone, minDeg, maxDeg);
     if(nUsed <= 0) return false;
     if(debug_timing){
          t = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
          printf("[INFO] is_ground_present() ---- took %.4lf ms (%.2lf Hz)\r\n", t*1000.0, (1.0/t));
     }

     if(best_slope) *best_slope = m;
     if(best_intercept) *best_intercept = b;

     if(visualize){
          cv::Mat display;
          cv::cvtColor(vmap, display, cv::COLOR_GRAY2BGR);
          int yf = int(vmap.cols * m) + b;
          cv::line(display, cv::Point(0, b), cv::Point(vmap.cols, yf), cv::Scalar(0,255,0), 2, CV_AA);
          cv::imshow("Estimated Gnd", display);
     }
     return true;
}


/** TODO: port to c++
def find_contours(self, _umap, threshold = 30.0, threshold_method = "perimeter", offset=(0,0), max_thresh=1500.0, debug=False):
"""
============================================================================
	Find contours in image and filter out those above a certain threshold
============================================================================
"""
umap = np.copy(_umap)
# try: umap = cv2.cvtColor(_umap,cv2.COLOR_BGR2GRAY)
# except: print("[WARNING] find_contours --- Unnecessary Image Color Converting")

_, contours, hierarchy = cv2.findContours(umap,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE,offset=offset)

if(threshold_method == "perimeter"):
  filtered_contours = [cnt for cnt in contours if cv2.arcLength(cnt,True) >= threshold]
  if max_thresh > 0: # Dont filter out big contours
      filtered_contours = [cnt for cnt in filtered_contours if cv2.arcLength(cnt,True) <= max_thresh]
  if(debug):
      raw_perimeters = np.array([cv2.arcLength(cnt,True) for cnt in contours])
      filtered_perimeters = np.array([cv2.arcLength(cnt,True) for cnt in filtered_contours])
      print("Raw Contour Perimeters:",np.unique(raw_perimeters))
      print("Filtered Contour Perimeters:",np.unique(filtered_perimeters))
elif(threshold_method == "area"):
  filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) >= threshold]
  if(debug):
      raw_areas = np.array([cv2.contourArea(cnt) for cnt in contours])
      filtered_areas = np.array([cv2.contourArea(cnt) for cnt in filtered_contours])
      print("Raw Contour Areas:",np.unique(raw_areas))
      print("Filtered Contour Areas:",np.unique(filtered_areas))
else:
  print("[ERROR] find_contours --- Unsupported filtering method!")

return filtered_contours,contours
*/


/** TODO:
def extract_contour_bounds(self, cnts, verbose=False, timing=False):
""" ===================================================================
	Attempt to find the horizontal bounds for detected contours
==================================================================== """
dt = 0
xBounds = []
disparityBounds = []

if(timing): t0 = time.time()

for cnt in cnts:
  try:
      x,y,rectw,recth = cv2.boundingRect(cnt)
      xBounds.append([x, x + rectw])
      disparityBounds.append([y, y + recth])
  except: pass

if(timing):
  t1 = time.time()
  dt = t1 - t0
  print("\t[INFO] extract_contour_bounds() --- Took %f seconds (%.2f Hz) to complete" % (dt, 1/dt))
return xBounds, disparityBounds, dt

*/


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
