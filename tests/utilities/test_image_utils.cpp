#include "utilities/image_utils.h"
#include "utilities/plot_utils.h"
#include "utilities/cv_utils.h"
#include "algorithms/vboats/vboats.h"
#include "algorithms/vboats/uvmap_utils.h"
#include <opencv2/core/utility.hpp>

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

// #define TEST_IMG_STRIPPING
// #define TEST_IMG_SPREADING
#define TEST_UVMAP_FILTERING
// #define TEST_UVMAP_GEN


void create_umap2(cv::Mat image, cv::Mat* umap, cv::Mat* vmap){
     // cv::Range ws(0,image.cols);
     // cv::Range hs(0, image.rows);
     double minVal, maxVal;
     cv::minMaxLoc(image, &minVal, &maxVal);
     int dmax = (int) maxVal + 1;

     cv::Mat umapMat = cv::Mat::zeros(dmax, image.cols, CV_8UC1);
     /** Size Temporarily transposed for ease of for-loop copying (Target size = h x dmax)*/
	cv::Mat vmapMat = cv::Mat::zeros(dmax, image.rows, CV_8UC1);
	// cv::Mat vmapMat = cv::Mat::zeros(image.rows, dmax, CV_8UC1);

     int channels[] = {0};
     int histSize[] = {dmax};
	float sranges[] = { 0, dmax };
	const float* ranges[] = { sranges };
     // cv::MatND histU, histV;

     // cv::MatND histU;
     double nthreads = 8.0;
     cv::parallel_for_(cv::Range(0,image.cols), [&](const cv::Range& range){
          cv::MatND histU;
          for(int i = range.start; i < range.end; i++){
     		cv::Mat uscan = image.col(i);
     		cv::calcHist(&uscan, 1, channels, cv::Mat(), histU, 1, histSize, ranges);
     		histU.col(0).copyTo(umapMat.col(i));
     	}
     }, nthreads);

     // cv::MatND histV;
     cv::parallel_for_(cv::Range(0, image.rows), [&](const cv::Range& range){
          cv::MatND histV;
          for(int j = range.start; j < range.end; j++){
     		cv::Mat vscan = image.row(j);
     		cv::calcHist(&vscan, 1, channels, cv::Mat(), histV, 1, histSize, ranges);
               histV.col(0).copyTo(vmapMat.col(j));
     	}
     }, nthreads);
     cv::Mat vmapTrans = vmapMat.t();
     // vmapMat = vmapMat.t();
     // cvinfo(vmapTrans,"vmapTrans");
     if(umap) *umap = umapMat;
     if(vmap) *vmap = vmapTrans;
}

using namespace std;

// struct ForEachOperator{
// 	uchar m_table[256];
// 	ForEachOperator(const uchar* const table){
// 		for(size_t i = 0; i < 256; i++){
// 			m_table[i] = table[i];
// 		}
// 	}
//
// 	void operator ()(uchar& p, const int * position) const{
// 		// Perform a simple operation
// 		p = m_table[p];
// 	}
// };
//
// // forEach use multiple processors, very fast
// cv::Mat& ScanImageAndReduce_forEach(cv::Mat& I, const uchar* const table){
// 	I.forEach<uchar>(ForEachOperator(table));
// 	return I;
// }

cv::Mat genDisparity(const cv::Mat depth){
     /** [INFO] CameraD415::get_depth_scale() --- Depth Scale = 0.001000
         [INFO] CameraD415::get_baseline() --- Baseline = 0.014732
         [INFO] CameraD415::get_intrinsics() --- Intrinsic Properties:
     	    Size ---------- [w, h]: 848, 480
     	    Focal Length -- [X, Y]: 596.39, 596.39
     	    Principle Point [X, Y]: 423.74, 242.01
     */
     float fxd = 596.39;
     float dscale = 0.001;
     float baseline = 0.014732;
     float trueMinDisparity = (fxd * baseline)/(0.1);
     float trueMaxDisparity = (fxd * baseline)/(10.0);
     // -----------------------
     cv::Mat dMeters, tmp, disparity8;
     double min, maxIn, maxDisparity, maxDepth;
     double minDIn, maxDIn;
     cv::minMaxLoc(depth, &minDIn, &maxDIn);
     cvinfo(depth,"depth input");

     double maxDepthMeters = maxDIn*(double)dscale;
     
     depth.convertTo(dMeters, CV_64F);
     tmp = tmp * dscale;
     // cvinfo(tmp,"depth input to CV_64F");

     cv::Mat disparity = cv::Mat((fxd * baseline) / tmp);
     // cvinfo(disparity,"disparity");
     cv::minMaxLoc(disparity, &min, &maxDisparity);
     double gain = 256.0 / maxDepth;
     double ratio = trueMinDisparity / maxDisparity;
     double offset = ratio / gain;
     double tmpgain = 256.0 / (double)(trueMinDisparity);
     int tmpVal = (tmpgain*maxDisparity);
     double delta = 256.0 / (double)(tmpVal);
     // float tmpgain = (float)(ratio*256.0) / this->_trueMinDisparity;
     // float tmpgain = 256.0 / maxDisparity;
     disparity.convertTo(disparity8,CV_8UC1, tmpgain);
     if(false){
          cvinfo(disparity8,"disparity8 pregain");
          printf(" --- tmpgain=%.2f | ratio=%.2f | maxDisparity=%.2f | delta=%.2f | tmpVal=%d --- \r\n",tmpgain, ratio, maxDisparity, delta,tmpVal);
          // disparity8.convertTo(disparity8,CV_8UC1, ratio);
          cv::Mat disparity88 = cv::Mat_<uchar>(disparity8 * delta);
          cvinfo(disparity88,"disparity8 postgain");
     }
     // disparity.convertTo(disparity8,CV_8UC1, 255.0/this->_trueMinDisparity);
     // cvinfo(tmpDisp,"tmpDisp");
     // cvinfo(disparity8,"disparity8");

     // cv::minMaxLoc(disparity8, &min, &maxIn);
     // double dmaxHat = maxIn * offset;
     // printf(" --- maxDisparity=%.2f | maxDepth=%.2f | trueMaxDisparity=%.2f | derivedMaxDepth=%.2f --- \r\n",maxDisparity, maxDepth, this->_trueMinDisparity, dmaxHat);

     // cv::imshow("disparity", disparity);
     // cv::imshow("tmpDisp", tmpDisp);
     // cv::imshow("disparity8", disparity8);
     return disparity8;
}

int main(int argc, char *argv[]){
     printf("[INFO] OpenCV ---- Num of cv threads = %d\r\n", cv::getNumThreads());
     printf("[INFO] OpenCV ---- Build Info:\r\n");
     std::cout << cv::getBuildInformation() << std::endl;
     printf(" ------------------------------------ \r\n");
     // cv::setNumThreads(2);

     int err;
     std::string fp;
     // const std::string imgDir = "/home/hyoung/devel/robo-commander/extras/data/uvmaps/img_util_vboats_tests/";
     // const std::string imgDir = "../../../extras/data/uvmaps/img_util_vboats_tests/";
     const std::string imgDir = "../extras/data/uvmaps/img_util_vboats_tests/";
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
     VBOATS vboats;

     std::string imgIdxStr;
     if(argc == 1){
          imgIdxStr = "2";
     } else imgIdxStr = argv[1];

     const std::string testDir = "../extras/data/uvmaps/processing_tests/";
     const std::string rgbPrefix = "raw_color_frame_";
     const std::string depPrefix = "raw_depth_frame_";
     const std::string dispPrefix = "ref_disparity_frame_";
     const std::string uPrefix = "ref_umap_frame_";
     const std::string vPrefix = "ref_vmap_frame_";

     std::string colorF = testDir + rgbPrefix + imgIdxStr + ".png";
     std::string depthF = testDir + depPrefix + imgIdxStr + ".png";
     std::string dispF = testDir + dispPrefix + imgIdxStr + ".png";
     std::string umapF = testDir + uPrefix + imgIdxStr + ".png";
     std::string vmapF = testDir + vPrefix + imgIdxStr + ".png";
     printf("Reading in image \'%s\'...\r\n",umapF.c_str());
     cv::Mat rgb = cv::imread(colorF);
     cv::Mat raw_depth = cv::imread(depthF, cv::IMREAD_UNCHANGED);
     cv::Mat refDisparity = cv::imread(dispF, cv::IMREAD_GRAYSCALE);
     cv::Mat umapSaved = cv::imread(umapF, cv::IMREAD_GRAYSCALE);
     cv::Mat vmapSaved = cv::imread(vmapF, cv::IMREAD_GRAYSCALE);
     cvinfo(raw_depth,"raw_depth_input");

     cv::Mat disparity;
     if(raw_depth.empty()) disparity = refDisparity.clone();
     else disparity = genDisparity(raw_depth);

     cv::Mat umap,vmap;
     genUVMapThreaded(disparity,&umap,&vmap);
     cvinfo(refDisparity,"reference disparity");
     cvinfo(disparity,"generated disparity");
     cvinfo(umap,"generated umap");
     cvinfo(vmap,"generated vmap");

     vector<Obstacle> obs;
     vboats.pipeline_disparity(disparity, umap, vmap, &obs);

     cv::namedWindow("raw color input", cv::WINDOW_AUTOSIZE ); cv::imshow("raw color input", rgb);
     cv::namedWindow("raw depth input", cv::WINDOW_AUTOSIZE ); cv::imshow("raw depth input", raw_depth);
     cv::namedWindow("reference disparity", cv::WINDOW_AUTOSIZE ); cv::imshow("reference disparity", refDisparity);
     cv::namedWindow("generated disparity", cv::WINDOW_AUTOSIZE ); cv::imshow("generated disparity", disparity);
     cv::namedWindow("reference vmap", cv::WINDOW_AUTOSIZE ); cv::imshow("reference vmap", vmapSaved);
     cv::namedWindow("reference umap", cv::WINDOW_AUTOSIZE ); cv::imshow("reference umap", umapSaved);
     cv::namedWindow("generated vmap", cv::WINDOW_AUTOSIZE ); cv::imshow("generated vmap", vmap);
     cv::namedWindow("generated umap", cv::WINDOW_AUTOSIZE ); cv::imshow("generated umap", umap);
     if(cv::waitKey(0) == 27) return 1;
#endif

     int nLoops = 1;
     double sum1 = 0.0, sum2 = 0.0, sum3 = 0.0, sum4 = 0.0;
     cv::Mat umap2,vmap2, umap3,vmap3, umap4,vmap4;
     double t, t1, t2, t3, dt, dt1, dt2, dt3;
#ifdef TEST_UVMAP_GEN
     printf(" =========================================== \r\n");
     for(int i = 0;i<nLoops;i++){
          printf("[INFO] UV Map Generation Performance Run %d:\r\n", i+1);
          // printf(" ------------------------------------------- \r\n");

          t = (double)cv::getTickCount();
          vboats.get_uv_map(disparity,&umap,&vmap);
          dt = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
          sum1 += dt*1000.0;
          printf("[INFO] ----- get_uv_map() ---- took %.4lf ms (%.2lf Hz)\r\n", dt*1000.0, (1.0/dt));
          // printf("[INFO] vboats.get_uv_map() ---- took %.4lf ms (%.2lf Hz)\r\n", dt*1000.0, (1.0/dt));

          t1 = (double)cv::getTickCount();
          vboats.get_uv_map_parallel(disparity,&umap2,&vmap2, 2.0);
          dt1 = ((double)cv::getTickCount() - t1)/cv::getTickFrequency();
          sum2 += dt1*1000.0;
          printf("[INFO] ----- get_uv_map_parallel() ---- took %.4lf ms (%.2lf Hz)\r\n", dt1*1000.0, (1.0/dt1));

          t2 = (double)cv::getTickCount();
          create_umap2(disparity,&umap3,&vmap3);
          dt2 = ((double)cv::getTickCount() - t2)/cv::getTickFrequency();
          sum3 += dt2*1000.0;
          printf("[INFO] ----- create_umap2() ---- took %.4lf ms (%.2lf Hz)\r\n", dt2*1000.0, (1.0/dt2));

          t3 = (double)cv::getTickCount();
          genUVMapThreaded(disparity,&umap4,&vmap4);
          dt3 = ((double)cv::getTickCount() - t3)/cv::getTickFrequency();
          sum4 += dt3*1000.0;
          printf("[INFO] ----- genUVMapThreaded() ---- took %.4lf ms (%.2lf Hz)\r\n", dt3*1000.0, (1.0/dt3));
          // printf("[INFO] create_umap2() ---- took %.4lf ms (%.2lf Hz)\r\n", dt2*1000.0, (1.0/dt2));
          printf(" ------------------------------------------- \r\n");
          // printf(" =========================================== \r\n");
     }
     double avg0 = sum1 / (double)nLoops;
     double avg1 = sum2 / (double)nLoops;
     double avg2 = sum3 / (double)nLoops;
     double avg3 = sum4 / (double)nLoops;
     printf("[INFO] UV Map Generation Avg Times ---- get_uv_map = %.4lf ms | get_uv_map_parallel = %.4lf ms | test2 = %.4lf ms | genUVMapThreaded = %.4lf ms\r\n", avg0,avg1,avg2, avg3);

     // cvinfo(umapSaved,"reference umap");
     // cvinfo(vmapSaved,"reference vmap");
     // cv::namedWindow("vmapTest", cv::WINDOW_AUTOSIZE ); cv::imshow("vmapTest", vmap2);
     // cv::namedWindow("umapTest", cv::WINDOW_AUTOSIZE ); cv::imshow("umapTest", umap2);
     // cv::namedWindow("vmapTest2", cv::WINDOW_AUTOSIZE ); cv::imshow("vmapTest2", vmap3);
     // cv::namedWindow("umapTest2", cv::WINDOW_AUTOSIZE ); cv::imshow("umapTest2", umap3);
     // // cv::namedWindow("equal vmap", cv::WINDOW_NORMAL ); cv::imshow("equal vmap", vTmp);
     // // cv::namedWindow("thresholded vmap", cv::WINDOW_NORMAL ); cv::imshow("thresholded vmap", vProcessed);
     // // cv::namedWindow("thresholded umap", cv::WINDOW_NORMAL ); cv::imshow("thresholded umap", uProcessed);
     // // cv::namedWindow("obstacles", cv::WINDOW_NORMAL ); cv::imshow("obstacles", clone);
     // if(cv::waitKey(0) == 27) return 1;
#endif

     return 1;
}
