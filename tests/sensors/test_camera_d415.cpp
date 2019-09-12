#include "sensors/camera_d415.h"
#include <iostream>

using namespace std;

int main(int argc, char *argv[]){
	CameraD415* cam = new CameraD415();
	cam->get_depth_scale(true);
	cam->get_intrinsics(true);
	cam->get_baseline(true);
	// namedWindow( "Trajectory", WINDOW_AUTOSIZE );// Create a window for display.

	printf("Press Ctrl+C to Stop...\r\n");
	// while(1){}
	vector<cv::Mat> imgs = cam->update_frames();

	cv::namedWindow("Trajectory", cv::WINDOW_AUTOSIZE );
	cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE );

	cv::Mat display;
	while(1){
		imgs = cam->update_frames();
		// if(imgs.size() <= 0){
		// 	cv::Mat depth = imgs.at(0);
		// 	cv::Mat rgb = imgs.at(1);
		// 	cv::imshow("Trajectory", rgb);
		// }
		cv::Mat depth = imgs.at(0);
		cv::Mat rgb = imgs.at(1);

		cv::equalizeHist( depth, depth );
		cv::applyColorMap(depth, display, cv::COLORMAP_JET);

		cv::imshow("Trajectory", rgb);
		cv::imshow("Depth", display);
		if(cv::waitKey(10) == 27){
			std::cout << "Esc key is pressed by user. Stoppig the video" << std::endl;
			break;
		}
	}

	delete cam;
     return 0;
}
