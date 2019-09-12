#include "sensors/camera_d415.h"
#include <iostream>

using namespace std;

int main(int argc, char *argv[]){
	CameraD415* cam = new CameraD415();
	cam->get_depth_scale(true);
	// namedWindow( "Trajectory", WINDOW_AUTOSIZE );// Create a window for display.

	printf("Press Ctrl+C to Stop...\r\n");
	// while(1){}

	delete cam;
     return 0;
}
