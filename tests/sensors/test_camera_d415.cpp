#include "sensors/camera_d415.h"
#include <iostream>

using namespace std;

typedef cv::Point_<uint8_t> Pixel;

void get_uv_map(const cv::Mat img, cv::Mat* _umap, cv::Mat* _vmap){
	cv::namedWindow("UMap", cv::WINDOW_NORMAL );
	cv::namedWindow("VMap", cv::WINDOW_NORMAL );
	double min, max;
	cv::minMaxLoc(img, &min, &max);

	int w = img.cols;
	int h = img.rows;
	int dmax = (int) max + 1;
	printf("max = %d, h = %d, w = %d\r\n",dmax,h,w);

	int hu = dmax; int wu = w; int hv = h; int wv = dmax;
	cv::Mat umap = cv::Mat::zeros(dmax, w, CV_8UC1);
	cv::Mat vmap = cv::Mat::zeros(h, dmax, CV_8UC1);
	printf("vmap h = %d, vmap w = %d\r\n",vmap.rows,vmap.cols);

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
	cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE );

	cv::Mat display, depth, rgb, disparity, disparity8, depth8, disp2;
	cv::Mat umap, vmap;
	int count = 0;
	double cvtGain;
	double minVal, maxVal;
	while(1){
		imgs = cam->read();
		rgb = imgs.at(0);
		depth = imgs.at(1);
		cv::Mat disparity = cam->convert_to_disparity(depth,cvtGain);
		// disparity = imgs.at(2);
		depth.convertTo(depth8,CV_8U,0.00390625);

		// disparity.convertTo(disparity8,CV_8U,cvtGain);
		// disparity.convertTo(disparity8,CV_8U);
		// depth.convertTo(disparity,CV_8U,0.00390625);
		// disparity.convertTo(disparity8,CV_8U,0.00390625);
		// for (int r = 0; r < disparity.rows; r++){
		// 	// Loop over all columns
		// 	for ( int c = 0; c < disparity.cols; c++){
		// 		// Obtain pixel at (r, c)
		// 		float val = disparity.at<uint8_t>(r, c) * 0.001;
		// 		// Pixel pixel = disparity.at<Pixel>(r, c);
		// 		// Apply complicatedTreshold
		// 		// complicatedThreshold(pixel);
		// 		// Put result back
		// 		uint8_t val8 = (uint8_t) (0.014579 * 386 / (float) val);
		// 		// printf(" %d", val8);
		// 		disparity.at<uint8_t>(r, c) = val8;
		// 	}
		// 	// printf("\r\n");
		// }
		// printf(" ------- \r\n");
		printf(" %.3f -- %.3f \r\n",minVal,maxVal);


		get_uv_map(disparity,&umap,&vmap);
		// if(count == 0){
		// 	// cv::equalizeHist(disparity, disparity8);
		// 	get_uv_map(disparity8,&umap,&vmap);
		// }
		printf(" ------- \r\n");
		// cv::Mat disparity = (0.014579 * 386) / depth8;
		// get_uv_map(disparity,&umap,&vmap);
		// get_uv_map(disparity,nullptr, nullptr);

		cv::equalizeHist(depth8, display);
		cv::applyColorMap(display, display, cv::COLORMAP_JET);
		// cv::applyColorMap(depth8, disp2, cv::COLORMAP_JET);
		// cv::applyColorMap(disparity8, disp2, cv::COLORMAP_JET);

		cv::imshow("RGB", rgb);
		// cv::imshow("Disparity", disparity);
		cv::imshow("Disparity8", disparity);

		cv::imshow("Depth8", depth8);
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
