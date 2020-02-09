#ifndef CAMERA_H_
#define CAMERA_H_

#include <string.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class Camera{
private:

     string _dev;
     int fast_threshold = 20;
	bool nonmaxSuppression = true;
public:
     VideoCapture* cap;
     Mat intrinsic_calib;
     Mat distortion_calib;
     Mat prev_img;
     Mat cur_img;
     float focal;
     cv::Point2d pp;

	Camera();
	Camera(const string& dev);
     ~Camera();

     int init(const string& dev);
     void update();
     void detect_features(Mat img, vector<KeyPoint>& keypts, vector<Point2f>& pts);
     void track_feautres(Mat img1, Mat img2, vector<Point2f>& pts1, vector<Point2f>& pts2);
     Mat get_raw_frame();
     Mat get_corrected_frame();
     Mat get_prepared_frame();

     Mat correct_frame(Mat frame);
     Mat greyscale_frame(Mat frame);
     Mat prepare_frame(Mat frame);

     void load_calibration(const string& file);

     void show_feed();

};

#endif /* CAMERA_H_*/
