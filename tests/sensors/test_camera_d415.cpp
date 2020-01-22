#include "sensors/camera_d415.h"
#include <iostream>
#include <unistd.h>                // For usleep
#include <chrono>

#include "utilities/cv_utils.h"
#include "utilities/image_utils.h"

using namespace chrono;
using namespace std;

int main(int argc, char *argv[]){
	bool do_processing = true;
	// CameraD415* cam = new CameraD415(480,640,30);
	int fps = 30;
	int rgb_resolution[2] = {848, 480};
	int depth_resolution[2] = {848, 480};
	// CameraD415* cam = new CameraD415(fps, rgb_resolution, fps, depth_resolution, true);
	CameraD415* cam = new CameraD415(fps, rgb_resolution, fps, depth_resolution);
	float dt_sleep = 1.0 / float(fps);
	cam->enable_alignment();
	// cam->enable_timing_debug();
	if(do_processing) cam->enable_filters();

	printf("Press Ctrl+C to Stop...\r\n");

	cv::Mat display, depth, rgb, disparity, disparity8, disparityAlt, disparityTest, depth8, depthNorm;
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
	bool debug_timing = true;
	if(threading) cam->start_thread();
	int errThread = 0;
	double t = (double)cv::getTickCount();
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
			int err = cam->read(&rgb, &depth);
			// printf("Image sizes: RGB (%d, %d, %d) type = %d -- Depth (%d, %d, %d) type = %d\r\n", rgb.cols,rgb.rows, rgb.channels(), rgb.type(), depth.cols,depth.rows, depth.channels(), depth.type());

			disparity = cam->convert_to_disparity(depth,&cvtGain, &cvtRatio);
			// now = high_resolution_clock::now();
			// time_span = duration_cast<duration<float>>(now - _prev_time);
			// dt = time_span.count();
			// printf(" read imgs --- %.7f ---- \r\n",dt);

			// printf(" %.3f -- %.3f \r\n",minVal,maxVal);

			// _prev_time = high_resolution_clock::now();
			// get_uv_map(disparity,&umap,&vmap);
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
				// if(debug_timing) t = (double)cv::getTickCount();
				// disparity = cam->convert_to_disparity(depth,&cvtGain, &cvtRatio);
				// // cvinfo(disparity,"disparityPrime");
				// if(debug_timing){
	               //      double dt = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
	               //      printf("[INFO] TestCameraD415() ---- convert_to_disparity step took %.2lf sec [%.2lf Hz]):\r\n", dt*1000, (1/dt));
	               // }
				// printf(" --------------------------- \r\n");
				// if(debug_timing) t = (double)cv::getTickCount();
				// disparityTest = cam->convert_to_disparity_alternative(depth,&cvtGain, &cvtRatio);
				// // cvinfo(disparityTest,"disparityTest");
				// if(debug_timing){
	               //      double dt = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
	               //      printf("[INFO] TestCameraD415() ---- convert_to_disparity_alternative step took %.2lf sec [%.2lf Hz]):\r\n", dt*1000, (1/dt));
	               // }
				// printf(" --------------------------- \r\n");
				// cv::imshow("RGB", rgb);
				// cv::imshow("Depth", depth);
				// cv::imshow("Disparity", disparity);
				// cv::imshow("DisparityTest", disparityTest);

				// cv::normalize(depth, depthNorm, 0, 255, cv::NORM_MINMAX, CV_8UC1);
				// cv::imshow("Depth Normalized", depthNorm);
				// cv::minMaxLoc(depth, &minVal, &maxVal);
				// depth.convertTo(depth8,CV_8U,(255.0/maxVal));
				// cv::applyColorMap(depth8, d1, cv::COLORMAP_JET);
				// cv::imshow("Depth Cmap", d1);
				// cv::normalize(disparity, display, 0, 255, cv::NORM_MINMAX, CV_8UC1);
				// cv::imshow("Disparity Norm", display);

				// if(debug_timing) t = (double)cv::getTickCount();
				// disparityAlt = cam->convert_to_disparity_test(depth,&cvtGain, &cvtRatio);
				// // cvinfo(disparityAlt,"disparityAlt");
				// if(debug_timing){
	               //      double dt = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
	               //      printf("[INFO] TestCameraD415() ---- convert_to_disparity_test step took %.2lf sec [%.2lf Hz]):\r\n", dt*1000, (1/dt));
	               // }
				// printf(" =========================== \r\n");
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
		usleep(dt_sleep * 1000000);
		count++;
	}

	delete cam;
     return 0;
}
