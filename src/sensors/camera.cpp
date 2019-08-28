#include "sensors/camera.h"
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

void Camera::detect_features(Mat img, vector<KeyPoint>& keypts, vector<Point2f>& pts){
     FAST(img, keypts, fast_threshold, nonmaxSuppression);
     KeyPoint::convert(keypts, pts, vector<int>());
}

void Camera::track_feautres(Mat img1, Mat img2, vector<Point2f>& pts1, vector<Point2f>& pts2){
     Size winSize = Size(21,21);
     TermCriteria termcrit = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);

     if(!pts1.empty()){
          vector<float> err;
          vector<uchar> status;
          calcOpticalFlowPyrLK(img1, img2, pts1, pts2, status, err, winSize, 3, termcrit, 0, 0.001);

          cout << "Size pts1, pts2: " << pts1.size() << ", " << pts2.size() << endl;

          // size_t i, k;
          // for(i = k = 0; i < pts2.size(); i++){
          //      if(!status[i])
          //           continue;
          //
          //      pts2[k++] = pts2[i];
          //      // circle(image, pts2[i], 3, Scalar(0,255,0), -1, 8);
          // }
          // cout << "i, k: " << i << ", " << k << endl;
          // pts2.resize(pts1.size());

          // Get rid of points where KLT tracking failed or those who have gone outside the frame
          int indexCorrection = 0;
          for(int i=0; i<status.size(); i++){
               Point2f pt = pts2.at(i- indexCorrection);
               if((status.at(i) == 0)||(pt.x<0)||(pt.y<0)){
                    if((pt.x<0)||(pt.y<0)){
                         status.at(i) = 0;
                    }
                    pts1.erase (pts1.begin() + i - indexCorrection);
                    pts2.erase (pts2.begin() + i - indexCorrection);
                    indexCorrection++;
               }
          }
          cout << "Size pts1, pts2: " << pts1.size() << ", " << pts2.size() << endl;
     }
}

Mat Camera::get_raw_frame(){
     Mat frame;
     cap->retrieve(frame);
     return frame;
}

Mat Camera::get_corrected_frame(){
     Mat frame, corrected_frame;
     cap->retrieve(frame);
     undistort(frame, corrected_frame, intrinsic_calib, distortion_calib);
     return corrected_frame;
}

Mat Camera::get_prepared_frame(){
     Mat frame, corrected_frame, img;
     cap->retrieve(frame);
     undistort(frame, corrected_frame, intrinsic_calib, distortion_calib);
     cvtColor(corrected_frame, img, COLOR_BGR2GRAY);
     return img;
}

Mat Camera::correct_frame(Mat frame){
     Mat img;
     undistort(frame, img, intrinsic_calib, distortion_calib);
     return img;
}

Mat Camera::greyscale_frame(Mat frame){
     Mat img;
     cvtColor(frame, img, COLOR_BGR2GRAY);
     return img;
}

Mat Camera::prepare_frame(Mat frame){
     Mat img, img2;
     img = correct_frame(frame);
     img2 = greyscale_frame(img);
     return img2;
}

void Camera::load_calibration(const string& file){
     Mat camMat, distMat;
     float fx, fy, x0, y0;

     cout << endl << "Reading: " << endl;
     FileStorage fs;
     fs.open(file, FileStorage::READ);

     fs["camera_matrix"] >> camMat;
     fs["distortion_coefficients"] >> distMat;

     intrinsic_calib = camMat;
     distortion_calib = distMat;

     fx = camMat.at<float>(0,0);
     fy = camMat.at<float>(1,1);
     focal = sqrt(fx*fx + fy*fy);

     x0 = camMat.at<float>(0,2);
     y0 = camMat.at<float>(1,2);
     cv::Point2d point(x0, y0);
     pp = point;
     cout << "focal = "<< focal << "   pp = " << pp << endl;

     cout << "camera_matrix = "<< endl << " "  << camMat << endl << endl;
     cout << "distortion_coefficients = "<< endl << " "  << distMat << endl << endl;
     fs.release();
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
