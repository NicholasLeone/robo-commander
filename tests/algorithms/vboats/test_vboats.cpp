#include "algorithms/vboats/vboats.h"
#include "sensors/camera_d415.h"

#include <iostream>
#include <chrono>

using namespace chrono;
using namespace std;

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

int main(int argc, char *argv[]){
	int count = 0;
	int err;
	int fps = 90;
	int rgb_resolution[2] = {848, 480};
	int depth_resolution[2] = {848, 480};
	// CameraD415* cam = new CameraD415(fps, rgb_resolution, fps, depth_resolution, true);
	CameraD415* cam = new CameraD415(60, rgb_resolution, fps, depth_resolution);
	cam->enable_alignment();
	// cam->enable_timing_debug();
	cam->enable_filters();

	VBOATS vb;
	printf("Press Ctrl+C to Stop...\r\n");

	cv::Mat depth, rgb, disparity, umap, vmap;
	cv::Mat* dummy = nullptr;

	cv::namedWindow("RGB", cv::WINDOW_AUTOSIZE );
	cv::namedWindow("Disparity", cv::WINDOW_AUTOSIZE );

	cam->start_thread();

	float dt;
	double cvtGain;
	high_resolution_clock::time_point _prev_time = high_resolution_clock::now();
	high_resolution_clock::time_point now;
	duration<float> time_span;
	while(1){
		err = cam->get_processed_queued_images(&rgb, &depth);
		if(err >= 0){
			now = high_resolution_clock::now();
			time_span = duration_cast<duration<float>>(now - _prev_time);
			dt = time_span.count();
			printf(" --- %.7f (%.2f) ---- \r\n",dt, (1/dt));
			disparity = cam->convert_to_disparity(depth,&cvtGain);
			vb.get_uv_map(disparity,&umap,&vmap, true, false);


			_prev_time = now;
			cv::imshow("RGB", rgb);
			cv::imshow("Disparity", disparity);
		}

		if(cv::waitKey(10) == 27){
			std::cout << "Esc key is pressed by user. Stoppig the video" << std::endl;
			break;
		}
		count++;
	}

	delete cam;
     return 0;
}
