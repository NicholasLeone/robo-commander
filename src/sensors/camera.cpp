#include "camera.h"
#include <iostream>

using namespace std;
using namespace cv;

Camera::Camera(){}

Camera::Camera(const string& dev){

     int err = init(dev);
     if(err < 0){
          exit(1);
     }
}

Camera::~Camera(){
     cap->release();
}

int Camera::init(const string& dev){
     this->cap = new VideoCapture(dev, CAP_V4L);

     if(!cap->isOpened()){
          printf("ERROR: Camera could not be found, or opened!\r\n");
          return -1;
     }

     printf("SUCCESS: Camera initialized!\r\n");
     return 1;
}

void Camera::update(){
     cap->grab();

     int frame_width = cap->get(CV_CAP_PROP_FRAME_WIDTH);
     int frame_height = cap->get(CV_CAP_PROP_FRAME_HEIGHT);
     int frame_time = cap->get(CV_CAP_PROP_POS_MSEC);
     int frame_fps = cap->get(CV_CAP_PROP_FPS);
     cout << "Resolution of the video : " << frame_width << " x " << frame_height << " | " << frame_time << " , " << frame_fps << endl;

}

Mat Camera::get_frame(){
     Mat frame;
     cap->retrieve(frame);
     return frame;
}

// void Camera::print_data(){
//      fflush(stdout);
//      printf("IMU DATA: \r\n");
//      printf("       Accelerations (m/s^2): %.4f        %.4f      %.4f\r\n", accel[0], accel[1], accel[2]);
//      printf("       Angular Velocities (rad/sec): %.4f        %.4f      %.4f\r\n", gyro[0], gyro[1], gyro[2]);
//      printf("       Magnetometer (μT): %.4f        %.4f      %.4f\r\n", mag[0], mag[1], mag[2]);
//      printf("       Fused Euler Angles (deg): %.4f        %.4f      %.4f\r\n\r\n", euler[0]*M_RAD2DEG, euler[1]*M_RAD2DEG, euler[2]*M_RAD2DEG);
// }

void Camera::show_feed(){}
