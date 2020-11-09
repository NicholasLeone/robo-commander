#include <stdio.h>
#include <iostream>

#include "base/definitions.h"
#include "utilities/utils.h"
#include "utilities/image_utils.h"

using namespace std;
namespace plt = matplotlibcpp;

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
     if(mat.empty()) return std::string();
     std::string r;
     int type = mat.type();
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
     if(mat.empty()) return std::string();
     std::string str = format("\'%s\' [%d, %d, %d, %s]", name, mat.cols, mat.rows, mat.channels(), cvtype2str(mat.type()).c_str());
     return str;
}
void cvinfo(const cv::Mat& mat, const char* label){
     if(!mat.empty()){
          double min, max;
          cv::minMaxLoc(mat, &min, &max);
          cv::Scalar mean,stddev;
          cv::meanStdDev(mat, mean, stddev);
          float mn = (float)(mean.val[0]);
          float sd = (float)(stddev.val[0]);

          if(!label) printf("%s -- Limits = [%.3lf, %.3lf] w/ mean = %.2f, std = %.2f\r\n",cvStrSize("Matrix",mat).c_str(), min, max, mn, sd);
          else printf("%s -- Limits = [%.3lf, %.3lf] w/ mean = %.2f, std = %.2f\r\n",cvStrSize(label,mat).c_str(), min, max, mn, sd);
     }
}
void print_cvmat(std::string header, const cv::Mat& mat, std::string rsep){
     if(!mat.empty()){
          printf("%s", header.c_str());
          for(int row = 0; row < mat.rows; ++row){
               if(row > 0) printf("%s",rsep.c_str());
               for(int col = 0; col < mat.cols; ++col){
                    printf("%.3lf, ",mat.at<double>(row,col));
               }
               if(row == mat.rows-1) printf("]\r\n");
               else printf("\r\n");
          }
     }
}

void spread_image(const cv::Mat& input, cv::Mat* output, const cv::Size& target_size, double* fx, double* fy, bool verbose){
     if(!input.empty()){
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
}
void spread_image(const cv::Mat& input, cv::Mat* output, double fx, double fy, cv::Size* new_size, bool verbose){
     if(!input.empty()){
          cv::Mat _output;
          cv::resize(input, _output, cv::Size(), fx, fy, cv::INTER_LINEAR);
          cv::Size _new_size = cv::Size(_output.cols, _output.rows);
          if(verbose) printf("[INFO] spread_image() --- Input Img (%d, %d) w/ factors (%.2f, %.2f) --> New Img (%d, %d) using cv::Size(%d, %d)\r\n",
               input.cols, input.rows, fx, fy, _output.cols, _output.rows, _new_size.width, _new_size.height);
          if(new_size) *new_size = _new_size;
          *output = _output;
     }
}
void unspread_image(const cv::Mat& input, cv::Mat* output, const cv::Size& target_size, float* fx, float* fy, bool verbose){
     if(!input.empty()){
          cv::Mat _output;
          double _fx = (double)(target_size.width/((double) input.cols));
          double _fy = (double)(target_size.height/((double) input.rows));
          cv::resize(input, _output, target_size, 0, 0, cv::INTER_AREA);
          if(verbose) printf("[INFO] unspread_image() --- Input Img (%d, %d) w/ Target Size (%d, %d) --> New Img (%d, %d) using factors (%.2f, %.2f)\r\n",
               input.cols, input.rows, target_size.width, target_size.height, _output.cols, _output.rows, _fx, _fy);
          if(fx) *fx = _fx;
          if(fy) *fy = _fy;
          *output = _output;
     }
}
void unspread_image(const cv::Mat& input, cv::Mat* output, float fx, float fy, cv::Size* new_size, bool verbose){
     if(!input.empty()){
          cv::Mat _output;
          cv::resize(input, _output, cv::Size(), fx, fy, cv::INTER_AREA);
          cv::Size _new_size = cv::Size(_output.cols, _output.rows);
          if(verbose) printf("[INFO] spread_image() --- Input Img (%d, %d) w/ factors (%.2f, %.2f) --> New Img (%d, %d) using cv::Size(%d, %d)\r\n",
               input.cols, input.rows, fx, fy, _output.cols, _output.rows, _new_size.width, _new_size.height);
          if(new_size) *new_size = _new_size;
          *output = _output;
     }
}
int strip_image(const cv::Mat& input, vector<cv::Mat>* strips, int nstrips, bool cut_horizontally, bool visualize, bool verbose){
     int i = 0;
     if(input.empty()){
          printf("[WARN] strip_image() --- Input image is empty, not stripping image.\r\n");
          return -1;
     }
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
          if(verbose) printf("[ERROR] strip_image() ---- \'w %% ncols = %d %% %d != 0\' Please choose another value for nstrips.\r\n", w, ncols);
          return -1;
     }else if(h % nrows != 0){
          if(verbose) printf("[ERROR] strip_image() ---- \'h %% nrows = %d %% %d != 0\' Please choose another value for nstrips.\r\n", h, nrows);
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
cv::Mat rotate_image(const cv::Mat& input, double angle){
     cv::Mat output;
     if(input.empty()) return output;
     cv::Point2f rotate_center(input.cols/2., input.rows/2.);
     cv::Mat RotMat = cv::getRotationMatrix2D(rotate_center, angle, 1.0);
     cv::warpAffine(input, output, RotMat, input.size());
     return output;
}

cv::Mat imCvtCmap(const cv::Mat& img){
     cv::Mat output;
     if(img.empty()) return output;
     cv::Mat input = img.clone();
     if(input.type() != CV_8UC1){
          if(input.channels() > 1) cv::cvtColor(input, input, cv::COLOR_BGR2GRAY);
          double minVal, maxVal;
          cv::minMaxLoc(input, &minVal, &maxVal);
          input.convertTo(output, CV_8UC1, (255.0/maxVal) );
     } else output = input;
     cv::applyColorMap(output, output, cv::COLORMAP_JET);
     return output;
}
int imshowCmap(const cv::Mat& img, std::string title){
     if(img.empty()) return -1;
     cv::namedWindow(title.c_str(), cv::WINDOW_NORMAL);
     cv::Mat display = imCvtCmap(img);
     if(!display.empty()) cv::imshow(title.c_str(), display);
     else return -2;
     return 0;
}

int pplot(cv::Mat image, std::string title, bool blocking){
     cv::Mat display;
     if(image.empty()) return -1;
     else display = image.clone();

     if(!title.empty()) plt::figure(title);
     else plt::figure(-1);

     std::vector<std::string> tks = {};
     std::map<std::string, double> figArgs;
     figArgs.insert(std::make_pair("top", 1.0)); figArgs.insert(std::make_pair("bottom", 0.0));
     figArgs.insert(std::make_pair("left", 0.0)); figArgs.insert(std::make_pair("right", 1.0));
     figArgs.insert(std::make_pair("wspace", 0.0)); figArgs.insert(std::make_pair("hspace", 0.0));
     plt::subplots_adjust(figArgs);

     std::map<std::string, std::string> imgArgs;
     imgArgs.insert(std::make_pair("interpolation", "bilinear"));
     plt::imshow(display, imgArgs);
     plt::xticks(tks); plt::yticks(tks);
     plt::show(blocking);
     return 0;
}
int pplots(std::vector<cv::Mat> images, long cols, long rows, std::string title, bool blocking){
     if(images.empty()) return -1;

     if(!title.empty()) plt::figure(title);
     else plt::figure(-1);

     std::vector<std::string> tks = {};
     std::map<std::string, std::string> imgArgs; imgArgs.insert(std::make_pair("interpolation", "bilinear"));
     std::map<std::string, double> figArgs;
     figArgs.insert(std::make_pair("top", 1.0)); figArgs.insert(std::make_pair("bottom", 0.0));
     figArgs.insert(std::make_pair("left", 0.0)); figArgs.insert(std::make_pair("right", 1.0));
     figArgs.insert(std::make_pair("wspace", 0.0)); figArgs.insert(std::make_pair("hspace", 0.0));

     long row = 0, col = 0;
     for(cv::Mat img : images){
          plt::subplot2grid(rows, cols, row, col);
          plt::subplots_adjust(figArgs);
          plt::xticks(tks); plt::yticks(tks);

          cv::Mat display;
          if(!img.empty()){
               display = img.clone();
               plt::imshow(display, imgArgs);
          }
          if(row < rows){ if(col < cols-1){ col++; } else{ row++; col = 0; } }
     }
     plt::show(blocking);
     return 0;
}

std::string img_to_str_simple(cv::Mat image, std::string encoding){
     if(image.empty()) return std::string();
     std::vector<uchar> buf;
     cv::imencode("." + encoding, image, buf);
     auto* enc_msg = reinterpret_cast<unsigned char*>(buf.data());
     std::string encoded = base64_encode(enc_msg, buf.size());
     return encoded;
}

std::string img_to_str(cv::Mat image, double* min, double* max, float* scale, bool preprocess, std::string encoding){
     if(image.empty()) return std::string();
     float scaleVal = 0.0;
     double minVal = 0.0, maxVal = 255.0;

     cv::Mat adjusted;
     if(image.type() != CV_32F) adjusted = image.clone();
     else{
          cv::Mat depthCopy = image.clone();
          if(preprocess){
               ForEachPrepareDepthConverter<float> preconverter;
               depthCopy.forEach<float>(preconverter);
          }

          if(strcmp(encoding.c_str(), "jpg") == 0){
               cv::minMaxIdx(depthCopy, &minVal, &maxVal);
               scaleVal = (255.0) / (maxVal-minVal);
               depthCopy.convertTo(adjusted, CV_8UC1, scaleVal, -minVal*scaleVal);
          } else adjusted = depthCopy.clone();
     }

     std::vector<uchar> buf;
     cv::imencode("." + encoding, adjusted, buf);
     auto* enc_msg = reinterpret_cast<unsigned char*>(buf.data());
     std::string encoded = base64_encode(enc_msg, buf.size());

     if(min) *min = minVal;
     if(max) *max = maxVal;
     if(scale) *scale = scaleVal;
     // printf("[INFO] img_to_str() --- Returning string size = %d.\n\r", (int)encoded.size());
     return encoded;
}
