#include "sensors/camera_d415.h"
#include <iostream>
#include <chrono>

using namespace chrono;
using namespace std;

typedef cv::Point_<uint8_t> Pixel;

// cv::Mat frame_to_mat(const rs2::frame& f){
//     using namespace cv;
//     using namespace rs2;
//
//     rs2::video_frame vf = f.as<rs2::video_frame>();
//     const int w = vf.get_width();
//     const int h = vf.get_height();
//
// 	if (f.get_profile().format() == RS2_FORMAT_BGR8){
// 		return cv::Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), cv::Mat::AUTO_STEP);
// 	} else if (f.get_profile().format() == RS2_FORMAT_RGB8){
// 		cv::Mat r = cv::Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), cv::Mat::AUTO_STEP);
// 		cv::cvtColor(r, r, COLOR_RGB2BGR);
// 		return r;
// 	} else if (f.get_profile().format() == RS2_FORMAT_Z16){
// 		return cv::Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
// 	} else if (f.get_profile().format() == RS2_FORMAT_Y8){
// 		return cv::Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
// 	} else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32){
// 		return cv::Mat(Size(w, h), CV_32FC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
// 	}
//
//     throw std::runtime_error("Frame format is not supported yet!");
// }
//
// // Converts depth frame to a matrix of doubles with distances in meters
// cv::Mat depth_frame_to_meters(const rs2::pipeline& pipe, const rs2::depth_frame& f){
//     cv::Mat dm = frame_to_mat(f);
//     dm.convertTo(dm, CV_64F);
//     double depth_scale = pipe.get_active_profile().get_device().first<rs2::depth_sensor>().get_depth_scale();
//     dm = dm * depth_scale;
//     return dm;
// }


void get_uv_map(const cv::Mat img, cv::Mat* _umap, cv::Mat* _vmap){
	cv::namedWindow("UMap", cv::WINDOW_NORMAL );
	cv::namedWindow("VMap", cv::WINDOW_NORMAL );
	double min, max;
	cv::minMaxLoc(img, &min, &max);

	int w = img.cols;
	int h = img.rows;
	int dmax = (int) max + 1;
	// printf("max = %d, h = %d, w = %d\r\n",dmax,h,w);

	int hu = dmax; int wu = w; int hv = h; int wv = dmax;
	cv::Mat umap = cv::Mat::zeros(dmax, w, CV_8UC1);
	cv::Mat vmap = cv::Mat::zeros(h, dmax, CV_8UC1);
	// printf("vmap h = %d, vmap w = %d\r\n",vmap.rows,vmap.cols);

	int histSize[] = {dmax};
	float sranges[] = { 0, dmax };
	const float* ranges[] = { sranges };
	int channels[] = {0};
	for(int i = 0; i < w; i++){
		cv::MatND hist;
		cv::Mat uscan = img.col(i);
		cv::calcHist(&uscan, 1, channels, cv::Mat(), hist, 1, histSize, ranges);
		// printf("Umap Column %d: h = %d, w = %d\r\n",i, hist.rows, hist.cols);
		hist.col(0).copyTo(umap.col(i));
	}
	for(int i = 0; i < h; i++){
		cv::MatND hist;
		cv::Mat vscan = img.row(i);
		cv::calcHist(&vscan, 1, channels, cv::Mat(), hist, 1, histSize, ranges);
		cv::Mat histrow = hist.t();
		// printf("VMap Row %d: h = %d, w = %d\r\n",i, histrow.rows, histrow.cols);
		histrow.row(0).copyTo(vmap.row(i));
	}

	cv::imshow("UMap", umap);
	cv::imshow("VMap", vmap);
}

int main(int argc, char *argv[]){
	CameraD415* cam = new CameraD415(480,640,30);
	cam->get_depth_scale(true);
	cam->get_intrinsics(true);
	cam->get_baseline(true);
	// namedWindow( "Trajectory", WINDOW_AUTOSIZE );// Create a window for display.

	printf("Press Ctrl+C to Stop...\r\n");

	vector<cv::Mat> imgs = cam->read();

	cv::namedWindow("RGB", cv::WINDOW_AUTOSIZE );
	// cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE );

	cv::Mat display, depth, rgb, disparity, disparity8, depth8, disp2;
	cv::Mat depth2, rgb2;
	cv::Mat umap, vmap;
	int count = 0;
	double cvtGain;
	float dt;
	double minVal, maxVal;
	high_resolution_clock::time_point _prev_time;
	high_resolution_clock::time_point now;
	duration<float> time_span;
	while(1){
		// _prev_time = high_resolution_clock::now();
		// imgs = cam->read();
		// rgb = imgs.at(0);
		// depth = imgs.at(1);
		// cv::Mat disparity = cam->convert_to_disparity(depth,cvtGain);
		// now = high_resolution_clock::now();
	     // time_span = duration_cast<duration<float>>(now - _prev_time);
	     // dt = time_span.count();
		// printf(" read vector --- %.7f ---- \r\n",dt);

		_prev_time = high_resolution_clock::now();
		cam->read(rgb, depth);
		cv::Mat disparity = cam->convert_to_disparity(depth,cvtGain);
		// now = high_resolution_clock::now();
	     // time_span = duration_cast<duration<float>>(now - _prev_time);
	     // dt = time_span.count();
		// printf(" read imgs --- %.7f ---- \r\n",dt);

		// printf(" %.3f -- %.3f \r\n",minVal,maxVal);

		// _prev_time = high_resolution_clock::now();
		get_uv_map(disparity,&umap,&vmap);
		now = high_resolution_clock::now();
	     time_span = duration_cast<duration<float>>(now - _prev_time);
	     dt = time_span.count();
		printf(" --- %.7f ---- \r\n",dt);
		// cv::Mat disparity = (0.014579 * 386) / depth8;
		// get_uv_map(disparity,&umap,&vmap);
		// get_uv_map(disparity,nullptr, nullptr);

		// depth.convertTo(depth8,CV_8U,0.00390625);
		// cv::equalizeHist(depth8, display);
		// cv::applyColorMap(display, display, cv::COLORMAP_JET);
		// cv::applyColorMap(depth8, disp2, cv::COLORMAP_JET);
		// cv::applyColorMap(disparity, disp2, cv::COLORMAP_JET);

		cv::imshow("RGB", rgb);
		// cv::imshow("Disparity", disparity);
		cv::imshow("Disparity8", disparity);

		// cv::imshow("Depth8", depth8);
		// cv::imshow("disp2", disp2);
		if(cv::waitKey(10) == 27){
			std::cout << "Esc key is pressed by user. Stoppig the video" << std::endl;
			break;
		}
		count++;
	}

	delete cam;
     return 0;
}
