#ifndef CAMERA_H_
#define CAMERA_H_

#include <string>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class Camera{
private:

     string _dev;

public:
     VideoCapture* cap;

	Camera();
	Camera(const string& dev);
     ~Camera();

     int init(const string& dev);
     void update();
     Mat get_frame();

     void show_feed();

};

#endif /* CAMERA_H_*/
