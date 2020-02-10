#include "sensors/camera_d415.h"
#include <iostream>
#include <unistd.h>                // For usleep
#include <chrono>

#include "utilities/cv_utils.h"
#include "utilities/image_utils.h"
// #include <pcl/point_types.h>

using namespace chrono;
using namespace std;

int main(int argc, char *argv[]){
	bool threading = false;
	bool debug_timing = true;
	bool do_processing = false;
	bool visualize = false;

	int fps = 15;
	int dfps = 60;
	int rgb_resolution[2] = {848, 480};
	int depth_resolution[2] = {848, 480};

	CameraD415* cam = new CameraD415(fps, rgb_resolution, fps, depth_resolution, !threading);

	int count = 0;
	int errThread;
	cv::Mat depth, rgb;
	float dt_sleep = 1.0 / float(fps);
	printf("[INFO] TestMinimalD415() ---- Sleeping %.2f secs [%d Hz] every loop\r\n", dt_sleep, fps);
	printf("Press Ctrl+C to Stop...\r\n");

	if(do_processing) cam->enable_filters();
	if(threading) cam->start_thread();

	double t = (double)cv::getTickCount();
	while(1){
		errThread = cam->get_processed_queued_images(&rgb, &depth);

		if(errThread >= 0){
			if(debug_timing){
                    double dt = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
                    printf("[INFO] TestCameraD415() ---- convert_to_disparity step took %.2lf ms [%.2lf Hz]):\r\n", dt*1000, (1/dt));
				t = (double)cv::getTickCount();
               }
			// printf(" --------------------------- \r\n");
			if(visualize){
				cv::imshow("RGB", rgb);
				cv::imshow("Depth", depth);
				if(cv::waitKey(10) == 27){
					std::cout << "Esc key is pressed by user. Stoppig the video" << std::endl;
					break;
				}
			}
		} else{}
		usleep(dt_sleep * 1000000);
		count++;
	}

	delete cam;
     return 0;
}
