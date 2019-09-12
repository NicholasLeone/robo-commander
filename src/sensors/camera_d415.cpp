#include <iostream>
#include "sensors/camera_d415.h"

using namespace std;

CameraD415::CameraD415(){}

CameraD415::CameraD415(const string& dev){}

CameraD415::~CameraD415(){}

int CameraD415::init(const string& dev){
     printf("SUCCESS: CameraD415 initialized!\r\n");
     return 1;
}

bool CameraD415::start(){
     return true;
}
bool CameraD415::stop(){
     return true;
}
bool CameraD415::reset(){
     return true;
}

void CameraD415::get_intrinsics(){}
void CameraD415::get_extrinsics(){}

float CameraD415::get_depth_scale(){
     float depth_scale = 0.0;
     return depth_scale;
}

cv::Mat CameraD415::get_rgb_image(){
     cv::Mat frame;
     return frame;
}

cv::Mat CameraD415::get_depth_image(){
     cv::Mat frame;
     return frame;
}

void CameraD415::grab_images(){
}

void CameraD415::update(){}
