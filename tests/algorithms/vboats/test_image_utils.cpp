#include <math.h>       /* ceil */

#include "algorithms/vboats/image_utils.h"

/** NOTES:

     spread_test:
          - crop umap height from  95 to 210
          - crop vmap width from 95 to 255
     me_and_farwall:
          - crop umap height from  40 to 130
          - crop vmap width from 40 to 255
     ondesk:
     -    crop umap height from  60 to 130
          - crop vmap width from 60 to 255
     underdesk:
     -    crop umap height from  70 to 255
          - crop vmap width from 70 to 255
*/


/**
depth.convertTo(dm, CV_64F);
cv::Mat tmp = dm*this->_dscale;
cv::Mat mask = cv::Mat(tmp == 0);
tmp.setTo(1, mask);
*/

void filter_disparity_vmap(const cv::Mat& input, cv::Mat* output, bool verbose, bool visualize){
     printf("%s\r\n",cvStrSize("Vmap Filtering Input",input).c_str());
     static const std::vector<float> thresholds = {0.15,0.15,0.01,0.01};
     // static const std::vector<float> thresholds = {0.85,0.85,0.75,0.5};
     int err;
     cv::Mat filtered;
     cv::Mat imgCopy = input.clone();
     int nThreshs = thresholds.size();

     int h = imgCopy.rows;
     int w = imgCopy.cols;
     int c = imgCopy.channels();
     if(visualize){
          cv::namedWindow("input", cv::WINDOW_NORMAL );
          cv::imshow("input", imgCopy);
          cv::waitKey(0);
     }

     double minVal, maxVal;
     cv::minMaxLoc(imgCopy, &minVal, &maxVal);

     int nStrips = int(ceil(maxVal/255.0) * nThreshs);
     int vMax = (int)maxVal;
     printf("vMax, nThreshs: %d, %d\r\n", vMax, nStrips);

     int divSection = 3;
     int dSection = int(h/float(divSection));

     cv::Rect topHalfRoi = cv::Rect(0,0,w,dSection);
     cv::Rect botHalfRoi = cv::Rect(0,dSection,w,h-dSection);
     if(verbose){
          std::cout << "Top Portion ROI: " << topHalfRoi << std::endl;
          std::cout << "Bottom Portion ROI: " << botHalfRoi << std::endl;
     }

     cv::Mat topHalf = imgCopy(topHalfRoi);
     cv::Mat botHalf = imgCopy(botHalfRoi);
     if(visualize){
          // cv::namedWindow("top portion", cv::WINDOW_NORMAL );
          // cv::namedWindow("bottom portion", cv::WINDOW_NORMAL );
          // cv::imshow("top portion", topHalf);
          // cv::imshow("bottom portion", botHalf);
          // cv::waitKey(0);
     }

     cv::minMaxLoc(topHalf, &minVal, &maxVal);
     int tmpThresh = int(maxVal*0.05);
     printf("tops max, tmpThresh: %d, %d\r\n", int(maxVal), tmpThresh);

     cv::Mat topThreshed, prefiltered;
     threshold(topHalf, topThreshed, tmpThresh, 255, cv::THRESH_TOZERO);
     // threshold(topHalf, topThreshed, tmpThresh, 255, cv::THRESH_BINARY);
     topThreshed.copyTo(imgCopy(topHalfRoi));
     if(visualize){
          // cv::namedWindow("thresholded portion", cv::WINDOW_NORMAL );
          // cv::namedWindow("thresholded portion", cv::WINDOW_NORMAL );
          // cv::imshow("thresholded portion", topThreshed);
          // cv::imshow("input image w/ thresholded portion", imgCopy);
          // cv::waitKey(0);
     }

     int idx = 0;
     cv::Scalar cvMean, cvStddev;
     std::vector<cv::Mat> strips;
     std::vector<cv::Mat> pStrips;
     err = strip_image(imgCopy, &strips, nStrips, true);
     for(cv::Mat strip : strips){
          cv::minMaxLoc(strip, &minVal, &maxVal);
          cv::meanStdDev(strip, cvMean, cvStddev);
          int tmpMax = (int) maxVal;
          double tmpMean = (double) cvMean[0];
          double tmpStd = (double) cvStddev[0];
          if(tmpMean == 0.0){
               pStrips.push_back(strip.clone());
               idx++;
               continue;
          }

          if(verbose) printf(" ---------- [Thresholding Strip %d] ---------- \r\n\tMax = %.1f, Mean = %.1f, Std = %.1f\r\n", idx,tmpMax,tmpMean,tmpStd);
          double maxValRatio = (double) vMax/255.0;
          double relRatio = (double)(tmpMax-tmpStd)/(double)vMax;
          double relRatio2 = (tmpMean)/(double)(tmpMax);
          if(verbose) printf("\tRatios: %.3f, %.3f, %.3f\r\n", maxValRatio, relRatio, relRatio2);

          double tmpGain;
          if(relRatio >= 0.4) tmpGain = relRatio + relRatio2;
          else tmpGain = 1.0 - (relRatio + relRatio2);

          int thresh = int(thresholds[idx] * tmpGain * tmpMax);
          if(verbose) printf("\tGain = %.2f, Thresh = %d\r\n", tmpGain,thresh);

          cv::Mat tmpStrip;
          threshold(strip, tmpStrip, thresh, 255,cv::THRESH_TOZERO);
          pStrips.push_back(tmpStrip.clone());
          if(visualize){
               // cv::imshow("strip", strip);
               // cv::imshow("thresholded strip", tmpStrip);
               // cv::waitKey(0);
          }
          idx++;
     }

     err = merge_strips(pStrips, &filtered);
     if(visualize){
          cv::namedWindow("reconstructed thresholded input", cv::WINDOW_NORMAL );
          cv::imshow("reconstructed thresholded input", filtered);
          cv::waitKey(0);
     }
     if(output) *output = filtered;
}

void filter_disparity_umap(const cv::Mat& input, cv::Mat* output, bool verbose, bool visualize){
     printf("%s\r\n",cvStrSize("Umap Filtering Input",input).c_str());
     static const std::vector<float> thresholds = {0.25,0.15,0.35,0.35};
     // static const std::vector<float> thresholds = {0.85,0.85,0.75,0.5};
     int err;
     cv::Mat filtered;
     cv::Mat imgCopy = input.clone();
     int nThreshs = thresholds.size();

     int h = imgCopy.rows;
     int w = imgCopy.cols;
     int c = imgCopy.channels();
     if(visualize){
          cv::namedWindow("input", cv::WINDOW_NORMAL );
          cv::imshow("input", imgCopy);
          cv::waitKey(0);
     }

     double minVal, maxVal;
     cv::minMaxLoc(imgCopy, &minVal, &maxVal);

     int uMax = (int)maxVal;
     int nStrips = int(ceil(maxVal/255.0) * nThreshs);
     printf("uMax, nThreshs: %d, %d\r\n", uMax, nStrips);

     threshold(imgCopy, imgCopy, 2, 255, cv::THRESH_TOZERO);

     int idx = 0;
     cv::Scalar cvMean, cvStddev;
     std::vector<cv::Mat> strips;
     std::vector<cv::Mat> pStrips;
     err = strip_image(imgCopy, &strips, nStrips, false);
     for(cv::Mat strip : strips){
          cv::minMaxLoc(strip, &minVal, &maxVal);
          cv::meanStdDev(strip, cvMean, cvStddev);
          int tmpMax = (int) maxVal;
          double tmpMean = (double) cvMean[0];
          double tmpStd = (double) cvStddev[0];
          if(tmpMean == 0.0){
               pStrips.push_back(strip.clone());
               idx++;
               continue;
          }

          if(verbose) printf(" ---------- [Thresholding Strip %d] ---------- \r\n\tMax = %.1f, Mean = %.1f, Std = %.1f\r\n", idx,tmpMax,tmpMean,tmpStd);
          double maxValRatio = (double) uMax/255.0;
          double relRatio = (double)(tmpMax-tmpStd)/(double)uMax;
          double relRatio2 = (tmpMean)/(double)(tmpMax);
          if(verbose) printf("\tRatios: %.3f, %.3f, %.3f\r\n", maxValRatio, relRatio, relRatio2);

          double tmpGain;
          if(relRatio >= 0.4) tmpGain = relRatio + relRatio2;
          else tmpGain = 1.0 - (relRatio + relRatio2);

          int thresh = int(thresholds[idx] * tmpGain * tmpMax);
          if(verbose) printf("\tGain = %.2f, Thresh = %d\r\n", tmpGain,thresh);

          cv::Mat tmpStrip;
          threshold(strip, tmpStrip, thresh, 255,cv::THRESH_TOZERO);
          pStrips.push_back(tmpStrip.clone());
          if(visualize){
               // cv::imshow("strip", strip);
               // cv::imshow("thresholded strip", tmpStrip);
               // cv::waitKey(0);
          }
          idx++;
     }

     err = merge_strips(pStrips, &filtered, false);
     if(visualize){
          cv::namedWindow("reconstructed thresholded input", cv::WINDOW_NORMAL );
          cv::imshow("reconstructed thresholded input", filtered);
          cv::waitKey(0);
     }

     if(output) *output = filtered;
}


int find_ground_line(const cv::Mat& vmap, vector<cv::Vec2f>* found_lines, int hough_thresh, double deg_offset){
     return 0;
}

int validate_ground_line(const vector<cv::Vec2f>& lines, double best_slope, int* best_intercept){
     return 0;
}


bool is_ground_present(const cv::Mat& vmap, double* best_slope, int* best_intercept, int hough_thresh, double gnd_deadzone, double minDeg, double maxDeg){
     cv::Mat display;
     cv::cvtColor(vmap, display, cv::COLOR_GRAY2BGR);
     // if(input.channels() == 3)

     vector<cv::Vec2f> lines;
     cv::HoughLines(vmap, lines, 1, CV_PI/180, hough_thresh);

     for(size_t i = 0; i < lines.size(); i++){
          float rho = lines[i][0], theta = lines[i][1];
          cv::Point pt1, pt2;
          double a = cos(theta), b = sin(theta);
          double x0 = a*rho, y0 = b*rho;
          pt1.x = cvRound(x0 + 1000*(-b));
          pt1.y = cvRound(y0 + 1000*(a));
          pt2.x = cvRound(x0 - 1000*(-b));
          pt2.y = cvRound(y0 - 1000*(a));
          cv::line(display, pt1, pt2, cv::Scalar(0,0,255), 3, CV_AA);
     }


     // flag = False
     // avgTheta = 0.0
     // cnt = summer = 0
     // minAng = np.deg2rad(minDeg)
     // maxAng = np.deg2rad(maxDeg)
     // angs = found_lines[:,0,1]
     // for theta in angs:
     //      angle = theta-np.pi
     //      if(minAng <= angle <= maxAng):
     //           summer += angle
     //           cnt += 1
     // if(cnt > 0):
     //      avgTheta = summer/float(cnt)
     //      flag = True
     return false;
}

// #define TEST_IMG_STRIPPING
// #define TEST_IMG_SPREADING
#define TEST_UVMAP_FILTERING

int main(int argc, char *argv[]){
     int err;
     std::string fp;
     const std::string imgDir = "/home/hyoung/devel/robo-commander/extras/data/uvmaps/img_util_vboats_tests/";
     const std::string disparityPrefix = "disparity_";
     const std::string umapPrefix = "raw_umap_";
     const std::string vmapPrefix = "raw_vmap_";
     if(argc == 1){
          fp = "spread_test";
          // fp = "/home/hunter/devel/robo-commander/extras/data/camera/pic.jpg";
     } else fp = argv[1];


#ifdef TEST_IMG_STRIPPING
     printf("Reading in image \'%s\'...\r\n",fp.c_str());
     cv::Mat image = cv::imread(fp);
     cv::imshow("Image", image);

     std::vector<cv::Mat> strips, stripsvert;
     err = strip_image(image, &strips, 5, false);
     err = strip_image(image, &stripsvert, 5, true);

     cv::Mat newImg, newImg2;
     err = merge_strips(strips, &newImg, false);
     cv::imshow("reconstructed image", newImg);
     cv::waitKey(0);

     err = merge_strips(stripsvert, &newImg2);
     cv::imshow("reconstructed image", newImg2);
     cv::waitKey(0);
#endif

#ifdef TEST_IMG_SPREADING
     std::string umapF = imgDir + umapPrefix + fp + ".png";
     std::string vmapF = imgDir + vmapPrefix + fp + ".png";
     printf("Reading in image \'%s\'...\r\n",umapF.c_str());
     cv::Mat umap = cv::imread(umapF, cv::IMREAD_GRAYSCALE);
     cv::Mat vmap = cv::imread(vmapF, cv::IMREAD_GRAYSCALE);

     cv::namedWindow("umap", cv::WINDOW_NORMAL );
     cv::namedWindow("vmap", cv::WINDOW_NORMAL );
     cv::imshow("umap", umap);
     cv::imshow("vmap", vmap);
     cv::waitKey(0);

     cv::Mat origU, origV, greyU, greyV;
     cv::cvtColor(umap, greyU, cv::COLOR_GRAY2BGR);
     cv::cvtColor(vmap, greyV, cv::COLOR_GRAY2BGR);
     cv::Mat copyU = greyU.clone();
     cv::Mat copyV = greyV.clone();
     cv::applyColorMap(umap, origU, cv::COLORMAP_JET);
     cv::applyColorMap(vmap, origV, cv::COLORMAP_JET);
     cv::imshow("orig umap", origU);
     cv::imshow("orig vmap", origV);
     cv::waitKey(0);

     int pux0, puy0, uw, uh;
     int pvx0, pvy0, vw, vh;
     pux0 = pvy0 = 0;
     uw = umap.cols; vh = vmap.rows;

     printf("%s\r\n",cvStrSize("Umap Input",umap).c_str());
     printf("%s\r\n",cvStrSize("Vmap Input",vmap).c_str());

     if(fp == "spread_test"){
          puy0 = 95; pvx0 = 95;
          uh = 210 - puy0; vw = 255 - pvx0;
     } else if(fp == "me_and_farwall"){
          puy0 = 40; pvx0 = 40;
          uh = 130 - puy0; vw = 255 - pvx0;
     } else if(fp == "ondesk"){
          puy0 = 60; pvx0 = 60;
          uh = 130 - puy0; vw = 255 - pvx0;
     } else if(fp == "underdesk"){
          puy0 = 70; pvx0 = 70;
          uh = 255 - puy0; vw = 255 - pvx0;
     } else{
          puy0 = 0; pvx0 = 0;
          uh = umap.rows; vw = vmap.cols;
     }

     cv::Rect mainURoi = cv::Rect(pux0, puy0, uw, uh);
     cv::Rect mainVRoi = cv::Rect(pvx0, pvy0, vw, vh);
     std::cout << "Umap ROI: " << mainURoi << std::endl;
     std::cout << "Vmap ROI: " << mainVRoi << std::endl;

     cv::Mat croppedUmap = umap(mainURoi);
     cv::Mat croppedVmap = vmap(mainVRoi);

     cv::rectangle(copyU, mainURoi, cv::Scalar(0, 0, 255), 1);
     cv::rectangle(copyV, mainVRoi, cv::Scalar(0, 0, 255), 1);
     cv::imshow("cropped region umap", copyU);
     cv::imshow("cropped region vmap", copyV);
     cv::waitKey(0);

     cv::Size uSpreadTargetSz = cv::Size(umap.size());
     cv::Size vSpreadTargetSz = cv::Size(vmap.size());
     cv::Size uUnspreadTargetSz = cv::Size(croppedUmap.size());
     cv::Size vUnspreadTargetSz = cv::Size(croppedVmap.size());

     cv::Mat dispU, dispV;
     // cv::cvtColor(croppedUmap, dispU, cv::COLOR_GRAY2BGR);
     // cv::cvtColor(croppedVmap, dispV, cv::COLOR_GRAY2BGR);
     cv::applyColorMap(croppedUmap, dispU, cv::COLORMAP_JET);
     cv::applyColorMap(croppedVmap, dispV, cv::COLORMAP_JET);

     printf("%s\r\n",cvStrSize("Umap Cropped",dispU).c_str());
     printf("%s\r\n",cvStrSize("Vmap Cropped",dispV).c_str());
     cv::namedWindow("cropped umap", cv::WINDOW_NORMAL );
     cv::namedWindow("cropped vmap", cv::WINDOW_NORMAL );
     cv::imshow("cropped umap", dispU);
     cv::imshow("cropped vmap", dispV);
     cv::waitKey(0);

     cv::Mat spreadUmap, spreadVmap;
     spread_image(dispU, &spreadUmap, uSpreadTargetSz, nullptr, nullptr, true);
     spread_image(dispV, &spreadVmap, vSpreadTargetSz, nullptr, nullptr, true);

     printf("%s\r\n",cvStrSize("Umap Crop Spread",spreadUmap).c_str());
     printf("%s\r\n",cvStrSize("Vmap Crop Spread",spreadVmap).c_str());
     cv::namedWindow("spread umap", cv::WINDOW_NORMAL );
     cv::namedWindow("spread vmap", cv::WINDOW_NORMAL );
     cv::imshow("spread umap", spreadUmap);
     cv::imshow("spread vmap", spreadVmap);
     cv::waitKey(0);

     cv::Mat unspreadUmap, unspreadVmap;
     spread_image(spreadUmap, &unspreadUmap, uUnspreadTargetSz, nullptr, nullptr, true);
     spread_image(spreadVmap, &unspreadVmap, vUnspreadTargetSz, nullptr, nullptr, true);

     printf("%s\r\n",cvStrSize("Umap Crop UnSpread",unspreadUmap).c_str());
     printf("%s\r\n",cvStrSize("Vmap Crop UnSpread",unspreadVmap).c_str());
     cv::namedWindow("unspread umap", cv::WINDOW_NORMAL );
     cv::namedWindow("unspread vmap", cv::WINDOW_NORMAL );
     cv::imshow("unspread umap", unspreadUmap);
     cv::imshow("unspread vmap", unspreadVmap);
     cv::waitKey(0);

     unspreadUmap.copyTo(greyU(mainURoi));
     unspreadVmap.copyTo(greyV(mainVRoi));

     printf("%s\r\n",cvStrSize("Result Umap",greyU).c_str());
     printf("%s\r\n",cvStrSize("Result Vmap",greyV).c_str());
     cv::namedWindow("result umap", cv::WINDOW_NORMAL );
     cv::namedWindow("result vmap", cv::WINDOW_NORMAL );
     cv::imshow("result umap", greyU);
     cv::imshow("result vmap", greyV);
     cv::waitKey(0);
#endif

#ifdef TEST_UVMAP_FILTERING
     std::string umapF = imgDir + umapPrefix + fp + ".png";
     std::string vmapF = imgDir + vmapPrefix + fp + ".png";
     printf("Reading in image \'%s\'...\r\n",umapF.c_str());
     cv::Mat umap = cv::imread(umapF, cv::IMREAD_GRAYSCALE);
     cv::Mat vmap = cv::imread(vmapF, cv::IMREAD_GRAYSCALE);

     // cv::namedWindow("umap", cv::WINDOW_NORMAL );
     // cv::namedWindow("vmap", cv::WINDOW_NORMAL );
     // cv::imshow("umap", umap);
     // cv::imshow("vmap", vmap);
     // cv::waitKey(0);

     cv::Mat vProcessed, uProcessed;
     filter_disparity_vmap(vmap, &vProcessed, true, true);
     filter_disparity_umap(umap, &uProcessed, true, true);
#endif

     return 1;
}
