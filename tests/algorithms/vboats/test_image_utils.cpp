#include "algorithms/vboats/image_utils.h"

int main(int argc, char *argv[]){
     int err;
     std::string fp;
     if(argc == 1) fp = "/home/hunter/devel/robo-commander/extras/data/camera/pic.jpg";
     else fp = argv[1];

     printf("Reading in image \'%s\'...\r\n",fp.c_str());
     cv::Mat image = cv::imread(fp);
     // cv::imshow("Image", image);

     std::vector<cv::Mat> strips, stripsvert;
     err = strip_image(image, &strips, 5, false);
     err = strip_image(image, &stripsvert, 5, true);

     // for(cv::Mat strip : strips){
     //      cv::imshow("strip", strip);
     //      cv::waitKey(0);
     // }
     // for(cv::Mat strip : stripsvert){
     //      cv::imshow("strip", strip);
     //      cv::waitKey(0);
     // }

     cv::Mat newImg, newImg2;
     err = merge_strips(strips, &newImg, false);
     cv::imshow("reconstructed image", newImg);
     cv::waitKey(0);

     err = merge_strips(stripsvert, &newImg2);
     cv::imshow("reconstructed image", newImg2);
     cv::waitKey(0);

     return 1;
}
