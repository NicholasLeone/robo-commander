#include "algorithms/vboats/vboats.h"

#include <chrono>

using namespace std;
using namespace chrono;

VBOATS::VBOATS(){}
VBOATS::~VBOATS(){}

void VBOATS::init(){}
void VBOATS::update(){}


float VBOATS::get_uv_map(const cv::Mat& image, cv::Mat* umap, cv::Mat* vmap,
     bool visualize, bool verbose, bool timing)
{
     float dt = -1.0;
     high_resolution_clock::time_point prev_time, now;
     if(timing) prev_time = high_resolution_clock::now();

     int w = image.cols;
     int h = image.rows;
     double min, max;
     cv::minMaxLoc(image, &min, &max);
     int dmax = (int) max + 1;

	int hu = dmax; int wu = w; int hv = h; int wv = dmax;
	cv::Mat _umap = cv::Mat::zeros(dmax, w, CV_8UC1);
	cv::Mat _vmap = cv::Mat::zeros(h, dmax, CV_8UC1);
     if(verbose){
          printf("max = %d, h = %d, w = %d\r\n",dmax,h,w);
          printf("vmap h = %d, vmap w = %d\r\n",_vmap.rows,_vmap.cols);
     }

     int channels[] = {0};
     int histSize[] = {dmax};
	float sranges[] = { 0, float(dmax) };
	const float* ranges[] = { sranges };
	for(int i = 0; i < w; i++){
		cv::MatND hist;
		cv::Mat uscan = image.col(i);
		cv::calcHist(&uscan, 1, channels, cv::Mat(), hist, 1, histSize, ranges);
		if(verbose) printf("Umap Column %d: h = %d, w = %d\r\n",i, hist.rows, hist.cols);
		hist.col(0).copyTo(_umap.col(i));
	}
	for(int i = 0; i < h; i++){
		cv::MatND hist;
		cv::Mat vscan = image.row(i);
		cv::calcHist(&vscan, 1, channels, cv::Mat(), hist, 1, histSize, ranges);
		cv::Mat histrow = hist.t();
		if(verbose) printf("VMap Row %d: h = %d, w = %d\r\n",i, histrow.rows, histrow.cols);
		histrow.row(0).copyTo(_vmap.row(i));
	}

     if(visualize){
          cv::namedWindow("UMap", cv::WINDOW_NORMAL );
          cv::namedWindow("VMap", cv::WINDOW_NORMAL );
          cv::imshow("UMap", _umap);
          cv::imshow("VMap", _vmap);
     }
     if(timing){
          // duration<float> time_span = duration_cast<duration<float>>(now - _prev_time);
          // dt = time_span.count();
          now = high_resolution_clock::now();
          dt = duration_cast<duration<float>>(now - prev_time).count();
     }
     return dt;
}
