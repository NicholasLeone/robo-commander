#include "sensors/camera.h"
#include "utils/utils.h"

using namespace std;

int flag_exit = 0;

void my_handler(int s){
	printf("Caught signal %d\n",s);
	flag_exit = 1;
}

int main(int argc, char *argv[]){
	cv::Mat frame;
	attach_CtrlZ(my_handler);
	string dev = "/dev/video1";
     Camera cam(dev);

	printf("Press Ctrl+Z to Stop...\r\n");
	while(!flag_exit){
		cam.update();
		frame = cam.get_frame();
		imshow("window_name", frame);
		if (waitKey(10) == 27){
			cout << "Esc key is pressed by user. Stoppig the video" << endl;
			break;
		}
     }
     return 0;
}
