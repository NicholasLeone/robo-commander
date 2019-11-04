#include "algorithms/vboats/vboats.h"
#include "sensors/camera_d415.h"
#include "algorithms/vboats/image_utils.h"
#include "algorithms/vboats/plot_utils.h"

#include <iostream>
#include <chrono>

using namespace chrono;
using namespace std;


int PlotGraph(cv::Mat& data){
     //converting the Mat to CV_64F
     data.convertTo(data, CV_64F);
     cv::Mat plot_result;
     cv::Ptr<cv::plot::Plot2d> plot = cv::plot::Plot2d::create(data);
     plot->setPlotBackgroundColor(cv::Scalar(50, 50, 50));
     plot->setPlotLineColor(cv::Scalar(50, 50, 255));
     plot->render(plot_result);
     cv::imshow("Graph", plot_result);
     // cv::waitKey(0);
     return 0;
}

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

	cv::Mat depth, rgb, disparity;
	cv::Mat depth8, rgb8, disparity8;
	cv::Mat d1, d2, d3;
	cv::Mat udisp, vdisp, disp, dispRaw;
	cv::Mat umap, vmap;
	cv::Mat* dummy = nullptr;

	// cv::namedWindow("RGB", cv::WINDOW_AUTOSIZE );
	// cv::namedWindow("Disparity", cv::WINDOW_AUTOSIZE );

	cam->start_thread();
	float dt;
	double tmpT, tmpDt;
	double cvtGain, cvtRatio;
	duration<float> time_span;
	high_resolution_clock::time_point now;
	high_resolution_clock::time_point _prev_time = high_resolution_clock::now();
     double t0, t1;
     bool verbose = false;
	bool try_thresholding = false;
	bool debug_timing = false;

     int morph_size = 10;
     int morph_size2 = 3;
     // cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*morph_size + 1, 2*morph_size+1), cv::Point(morph_size, morph_size));
     cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*morph_size + 1, 2*morph_size2+1), cv::Point(morph_size, morph_size2));

	while(1){
		err = cam->get_processed_queued_images(&rgb, &depth);
		if(err >= 0){
               t1 = (double)cv::getTickCount();
			// now = high_resolution_clock::now();

			// if(verbose) cvinfo(depth,"DepthIn");
			cv::Mat tmp, tmp2;
			double minVal, maxVal, dmax;
			double dtypeGain;
			// try{
			//      cv::minMaxLoc(depth, &minVal, &maxVal);
			// 	printf("Depth before min, max = %.2f, %.2f\r\n",minVal, maxVal);
			// 	dtypeGain = (255.0/maxVal);
			// 	depth.convertTo(tmp, CV_8U, dtypeGain);
			// 	cv::minMaxLoc(tmp, &minVal, &maxVal);
			// 	printf("Depth after min, max = %.2f, %.2f\r\n",minVal, maxVal);
			// 	cv::imshow("Depth", tmp);
			// 	cv::waitKey(0);
			// } catch(cv::Exception& e){ printf("A standard exception was caught, with message \'%s\'.\r\n", e.what()); }

			// depth.convertTo(depth8,CV_8U, (255.0/65535.0));
			// // depth.convertTo(depth8,CV_8U,(1.0/256.0));
			// printf("%s\r\n",cvStrSize("Depth8",depth8).c_str());
			// try{
			// 	cv::applyColorMap(depth8, disp, cv::COLORMAP_JET);
			// 	cv::imshow("Depth8", disp);
			// 	cv::waitKey(0);
			// } catch(cv::Exception& e){ printf("A standard exception was caught, with message \'%s\'.\r\n", e.what()); }
               if(debug_timing) tmpT = (double)cv::getTickCount();
			// cv::minMaxLoc(depth, &minVal, &dmax);
			disparity = cam->convert_to_disparity(depth,&cvtGain, &cvtRatio);
			// cv::minMaxLoc(disparity, &minVal, &maxVal);
               if(debug_timing){
                    tmpDt = ((double)cv::getTickCount() - tmpT)/cv::getTickFrequency();
                    printf("[INFO] disparity conversion() ---- took %.4lf ms (%.2lf Hz)\r\n", tmpDt*1000.0, (1.0/tmpDt));
               }
               // cvinfo(disparity,"Disparity");
			// if(verbose) printf("disparity min, max = %.2f, %.2f --- ratio = %.3f\r\n",minVal, maxVal, cvtRatio);
			// disparity = cam->convert_to_disparity(depth,&cvtGain);
			// disparity = cam->convert_to_disparity(depth8,&cvtGain);
			// if(verbose) printf("%s\r\n",cvStrSize("Disparity",disparity).c_str());



			// disparity.convertTo(disparity8,CV_8U);
			// disparity.convertTo(disparity8,CV_8U,(1.0/256.0));
			// printf("%s\r\n",cvStrSize("Disparity8",disparity8).c_str());
			// try{
			// 	cv::applyColorMap(disparity8, disp, cv::COLORMAP_JET);
			// 	cv::imshow("Disparity8", disp);
			// 	cv::waitKey(0);
			// } catch(cv::Exception& e){ printf("A standard exception was caught, with message \'%s\'.\r\n", e.what()); }

			// disparity = cam->convert_to_disparity(depth,&cvtGain);
               if(debug_timing) tmpT = (double)cv::getTickCount();
			vb.get_uv_map(disparity,&umap,&vmap, false, "raw");
			// printf("%s --- %s\r\n", cvStrSize("Umap",umap).c_str(), cvStrSize("Vmap",vmap).c_str());
               if(debug_timing){
                    tmpDt = ((double)cv::getTickCount() - tmpT)/cv::getTickFrequency();
                    printf("[INFO] UV Map generation() ---- took %.4lf ms (%.2lf Hz)\r\n", tmpDt*1000.0, (1.0/tmpDt));
               }

               if(debug_timing) tmpT = (double)cv::getTickCount();
			vector<Obstacle> obs;
		     // pipeline_disparity(disparity, umap, vmap, &obs, &element);
		     pipeline_disparity(disparity, umap, vmap, &obs);
               if(debug_timing){
                    tmpDt = ((double)cv::getTickCount() - tmpT)/cv::getTickFrequency();
                    printf("[INFO] pipeline_disparity() ---- took %.4lf ms (%.2lf Hz)\r\n", tmpDt*1000.0, (1.0/tmpDt));
               }

			// cv::Scalar mean, stddev;
		     // cv::meanStdDev(disparity,mean, stddev);
		     // cv::Mat stdImg = (mean-disparity)/stddev;
			// vb.get_uv_map(disparity,&umap,&vmap, true, "standardized");
			// time_span = duration_cast<duration<float>>(now - _prev_time);
               // _prev_time = now;
			// dt = time_span.count();
               tmpDt = ((double)cv::getTickCount() - t1)/cv::getTickFrequency();
               printf("[INFO] Vboats pipeline() ---- took %.4lf ms (%.2lf Hz)\r\n", tmpDt*1000.0, (1.0/tmpDt));
			// printf("[INFO] Vboats pipeline --- took %.4lf ms (%.2lf Hz)\r\n", dt*1000.0, (1.0/dt));
			printf(" -------------------------------- \r\n");
			// try{
			// 	cv::applyColorMap(disparity, disp, cv::COLORMAP_JET);
			// 	cv::imshow("Disparity", disp);
			// 	cv::convertScaleAbs(depth, dispRaw, 255 / dmax);
			// 	cv::applyColorMap(dispRaw, dispRaw, cv::COLORMAP_JET);
			// 	cv::imshow("Depth", dispRaw);
			// 	// cv::waitKey(0);
			// } catch(cv::Exception& e){ printf("A standard exception was caught, with message \'%s\'.\r\n", e.what()); }

               // cv::applyColorMap(umap, udisp, cv::COLORMAP_JET);
               // cv::applyColorMap(vmap, vdisp, cv::COLORMAP_JET);
			// cv::imshow("RGB", rgb);
			// cv::imshow("Umap", udisp);
			// cv::imshow("Vmap", vdisp);

			// cv::imshow("Disparity", disparity);
			// cv::waitKey(0);
		}

		if(cv::waitKey(1) == 27){
			std::cout << "Esc key is pressed by user. Stoppig the video" << std::endl;
			break;
		}
		count++;
	}

	delete cam;
     return 0;
}
