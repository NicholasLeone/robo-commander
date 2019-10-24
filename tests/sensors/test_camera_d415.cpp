#include "sensors/camera_d415.h"
#include <iostream>
#include <unistd.h>                // For usleep
#include <chrono>

#include "algorithms/vboats/image_utils.h"

using namespace chrono;
using namespace std;

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
	bool do_processing = true;
	// CameraD415* cam = new CameraD415(480,640,30);
	int fps = 90;
	int rgb_resolution[2] = {848, 480};
	int depth_resolution[2] = {848, 480};
	// CameraD415* cam = new CameraD415(fps, rgb_resolution, fps, depth_resolution, true);
	CameraD415* cam = new CameraD415(60, rgb_resolution, fps, depth_resolution);
	cam->enable_alignment();
	cam->enable_timing_debug();
	if(do_processing) cam->enable_filters();

	// namedWindow( "Trajectory", WINDOW_AUTOSIZE );// Create a window for display.

	printf("Press Ctrl+C to Stop...\r\n");

	// cv::Mat rgbRaw, depthRaw, dummy;
	// // int err = cam->read(&rgbRaw, &depthRaw, cv::Mat(), true);
	// int err = cam->read(&rgbRaw, &depthRaw, true);
	// vector<cv::Mat> imgs = cam->read(true);

	// cv::namedWindow("RGB", cv::WINDOW_AUTOSIZE );
	// cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE );

	cv::Mat display, depth, rgb, disparity, disparity8, disparityAlt, disparityAlt8, depth8, depthNorm;
	cv::Mat depthraw, rgbraw, disparityraw;
	cv::Mat d1, d2, d3, processed;
	cv::Mat umap, vmap;
	int count = 0;
	double cvtGain, cvtRatio;
	float dt;
	double minVal, maxVal;
	high_resolution_clock::time_point _prev_time;
	high_resolution_clock::time_point now;
	duration<float> time_span;
	bool threading = true;
	if(threading) cam->start_thread();
	int errThread = 0;
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
		if(!threading){
			_prev_time = high_resolution_clock::now();
			int err = cam->read(&rgb, &depth, &processed, true, do_processing);
			// printf("Image sizes: RGB (%d, %d, %d) type = %d -- Depth (%d, %d, %d) type = %d\r\n", rgb.cols,rgb.rows, rgb.channels(), rgb.type(), depth.cols,depth.rows, depth.channels(), depth.type());

			disparity = cam->convert_to_disparity(depth,&cvtGain, &cvtRatio);
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
			printf(" --- %.7f (%.2f) ---- \r\n",dt, (1/dt));
			// cv::Mat disparity = (0.014579 * 386) / depth8;
			// get_uv_map(disparity,&umap,&vmap);
			// get_uv_map(disparity,nullptr, nullptr);

			// depth.convertTo(depth8,CV_8U,0.00390625);
			// cv::equalizeHist(depth8, display);
			// cv::applyColorMap(display, display, cv::COLORMAP_JET);
			// cv::applyColorMap(depth8, disp2, cv::COLORMAP_JET);
			// cv::applyColorMap(disparity, disp2, cv::COLORMAP_JET);

			cv::imshow("RGB", rgb);
			cv::imshow("Depth", depth);
			cv::imshow("Disparity", disparity);
			if(do_processing) cv::imshow("Processed", processed);
			if(cv::waitKey(10) == 27){
				std::cout << "Esc key is pressed by user. Stoppig the video" << std::endl;
				break;
			}
		} else{
			// cam->get_queued_images(&rgb, &depth, &processed);
			// printf("Image sizes: RGB (%d, %d, %d) type = %d -- Depth (%d, %d, %d) type = %d\r\n", rgb.cols,rgb.rows, rgb.channels(), rgb.type(), depth.cols,depth.rows, depth.channels(), depth.type());
			errThread = cam->get_processed_queued_images(&rgb, &depth);
			if(errThread >= 0){
				cv::minMaxLoc(depth, &minVal, &maxVal);
				cv::imshow("RGB", rgb);
				disparity = cam->convert_to_disparity(depth,&cvtGain, &cvtRatio);
				cv::normalize(disparity, display, 0, 255, cv::NORM_MINMAX, CV_8UC1);
				cv::normalize(depth, depthNorm, 0, 255, cv::NORM_MINMAX, CV_8UC1);
				depth.convertTo(depth8,CV_8U,(255.0/maxVal));
				cv::applyColorMap(depth8, d1, cv::COLORMAP_JET);
				cv::imshow("Depth", depth);
				cv::imshow("Depth Cmap", d1);
				cv::imshow("Depth Normalized", depthNorm);
				cv::imshow("Disparity", disparity);
				cv::imshow("Disparity Norm", display);
			}

			// errThread = cam->get_raw_queued_images(&rgbraw, &depthraw);
			// if(errThread >= 0){
			// 	disparityraw = cam->convert_to_disparity(depthraw,&cvtGain);
			// 	// depth.convertTo(depth8,CV_8U,(1.0/256.0));
			// 	// disparity.convertTo(disparity8,CV_8U,(1.0/256.0));
			//
			// 	cv::imshow("RGB Raw", rgbraw);
			// 	// cv::applyColorMap(depth8, d1, cv::COLORMAP_JET);
			// 	// cv::applyColorMap(disparity, d2, cv::COLORMAP_JET);
			// 	cv::imshow("Depth Raw", depthraw);
			// 	cv::imshow("Disparity Raw", disparityraw);
			// 	// cv::imshow("DisparityAlt", disparity8);
			// }
			if(cv::waitKey(10) == 27){
				std::cout << "Esc key is pressed by user. Stoppig the video" << std::endl;
				break;
			}
		}

		// cv::imshow("Depth8", depth8);
		// cv::imshow("disp2", disp2);
		count++;
	}

	delete cam;
     return 0;
}
