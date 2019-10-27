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

#include "algorithms/vboats/plot_utils.h"
// #define TEST_IMG_STRIPPING
// #define TEST_IMG_SPREADING
#define TEST_UVMAP_FILTERING

int PlotGraph(cv::Mat& data){
     //converting the Mat to CV_64F
     data.convertTo(data, CV_64F);
     cv::Mat plot_result;
     cv::Ptr<cv::plot::Plot2d> plot = cv::plot::Plot2d::create(data);
     plot->setPlotBackgroundColor(cv::Scalar(50, 50, 50));
     plot->setPlotLineColor(cv::Scalar(50, 50, 255));
     plot->render(plot_result);
     cv::imshow("Graph", plot_result);
     cv::waitKey(0);
     return 0;
}

/**
[INFO] CameraD415::get_depth_scale() --- Depth Scale = 0.001000
[INFO] CameraD415::get_baseline() --- Baseline = 0.014732
[INFO] CameraD415::get_intrinsics() --- Intrinsic Properties:
	Size ---------- [w, h]: 848, 480
	Focal Length -- [X, Y]: 596.39, 596.39
	Principle Point [X, Y]: 423.74, 242.01
     K = [596.391845703125, 0, 423.7422790527344;
           0, 596.391845703125, 242.0074920654297;
           0, 0, 1
     ]
*/


int main(int argc, char *argv[]){
     int err;
     std::string fp;
     // const std::string imgDir = "/home/hyoung/devel/robo-commander/extras/data/uvmaps/img_util_vboats_tests/";
     const std::string imgDir = "../../../extras/data/uvmaps/img_util_vboats_tests/";
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
     std::string dispF = imgDir + disparityPrefix + fp + ".png";
     std::string umapF = imgDir + umapPrefix + fp + ".png";
     std::string vmapF = imgDir + vmapPrefix + fp + ".png";
     printf("Reading in image \'%s\'...\r\n",umapF.c_str());
     cv::Mat disparity = cv::imread(dispF, cv::IMREAD_GRAYSCALE);
     cv::Mat umapSaved = cv::imread(umapF, cv::IMREAD_GRAYSCALE);
     cv::Mat vmapSaved = cv::imread(vmapF, cv::IMREAD_GRAYSCALE);

     // vector<float> vthreshs = {0.15,0.15,0.01,0.01};
     // vector<float> vthreshs = {0.85,0.85,0.75,0.5};
     // vector<float> vthreshs = {0.45, 0.45,0.35,0.25};
     vector<float> vthreshs = {0.35, 0.35,0.25,0.35};

     vector<float> uthreshs = {0.25,0.15,0.35,0.35};
     // vector<float> thresholds = {0.85,0.85,0.75,0.5};

     cv::Mat vTmp, uTmp;
     cv::Mat vProcessed, uProcessed;
     filter_disparity_umap(umapSaved, &uProcessed, &uthreshs);
     filter_disparity_vmap(vmapSaved, &vProcessed, &vthreshs);

     cv::equalizeHist(vmapSaved, vTmp);
     cv::namedWindow("vmap", cv::WINDOW_NORMAL ); cv::imshow("vmap", vmapSaved);
     cv::namedWindow("equal vmap", cv::WINDOW_NORMAL ); cv::imshow("equal vmap", vTmp);
     cv::namedWindow("thresholded vmap", cv::WINDOW_NORMAL ); cv::imshow("thresholded vmap", vProcessed);

     cv::namedWindow("umap", cv::WINDOW_NORMAL ); cv::imshow("umap", umapSaved);
     cv::namedWindow("thresholded umap", cv::WINDOW_NORMAL ); cv::imshow("thresholded umap", uProcessed);

     cv::Mat vertMat, horzMat;
     cvinfo(vProcessed, "processedV");
     // cv::reduce(vProcessed,vertMat,1,CV_REDUCE_SUM, CV_32S);
     int hist_w = 512; int hist_h = 400;
     cv::reduce(vProcessed,horzMat,0,CV_REDUCE_SUM, CV_32S);
     cv::Mat histImage = cv::Mat(hist_h, hist_w, CV_8UC3, cv::Scalar(0,0,0));
     cv::Mat histNormImg = cv::Mat(hist_h, hist_w, CV_8UC3, cv::Scalar(0,0,0));
     cvinfo(horzMat, "horzMat");
     PlotGraph(horzMat);
     // printf("%d rows, %")
     // plt::plot(horzMat);
     // plt::show();
     // cv::normalize(horzMat, histNorm, 0, histImage.rows, cv::NORM_MINMAX, CV_8UC1);

     // for(int i = 1; i < horzMat.cols; i++){
     //      // for pixel in rng:
	// 	// 	if depth == None:
	// 	// 		histVal = hist[pixel]
     //      //
	// 	// 	if histVal == 0: zeroCount += 1
     //
	// 		// if zeroCount >= zeroFlag and leftFound == False:
	// 		// 	leftFound = True
	// 		// 	xmidL = pixel + xbuffer
	// 		// if leftFound == True and histVal > 0 and rightFound == False:
	// 		// 	rightFound = True
	// 		// 	xmidR = pixel + xbuffer
	// 		// if leftFound == True and rightFound == True:
	// 		// 	xmid = np.array([xmidL, xmidR])
	// 		// 	# print("Found both sides at " , xmid)
	// 		// 	break
     // }
     double t = (double)cv::getTickCount();;

     float gndM; int gndB;
     float* line_params;
     bool gndPresent = is_ground_present(vProcessed, &gndM,&gndB);
     if(gndPresent){
          float tmpParams[] = {gndM, (float) gndB};
          line_params = &tmpParams[0];
          printf("Ground Found\r\n");
     } else{
          line_params = nullptr;
          printf("No Ground Found\r\n");
     }
     vector<vector<cv::Point>> contours;
     find_contours(uProcessed, &contours);
     int nCnts = contours.size();
     // vector<int> xlims = {100, 110};
     int nObs = 0;
     vector<int> xLims;
     vector<int> dLims;
     vector<int> yLims;

     bool verbose = true;
     for(int i = 0; i < nCnts; i++){
          vector<cv::Point> contour = contours[i];
          extract_contour_bounds(contour,&xLims, &dLims);
          int nWins = obstacle_search_disparity(vProcessed,dLims, &yLims, nullptr, nullptr, line_params);
          if(nWins == 0) continue;
          if((yLims.size() <= 2) && (gndPresent)){
               if(verbose) printf("[INFO] Found obstacle with zero height. Skipping...\r\n");
               continue;
          } else if(yLims.size() <= 1){
               if(verbose) printf("[INFO] Found obstacle with zero height. Skipping...\r\n");
               continue;
          } else if(yLims[0] == yLims.back()){
               if(verbose) printf("[INFO] Found obstacle with zero height. Skipping...\r\n");
               continue;
          }
          if(verbose) printf("[INFO] Adding Obstacle (%d y limits)...\r\n",yLims.size());
          nObs++;
          // ybounds.append(ys)
          //  obs.append([
          //      (xs[0],ys[0]),
          //      (xs[1],ys[-1])
          //  ])
          //  obsUmap.append([
          //      (xs[0],ds[0]),
          //      (xs[1],ds[1])
          //  ])
          //  windows.append(ws)
          //  dBounds.append(ds)
          if(verbose) printf(" --------------------------- \r\n");
     }

     t = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
     printf("[INFO] find_obstacles() ---- took %.4lf ms (%.2lf Hz) to find %d obstacles\r\n", t*1000.0, (1.0/t), nObs);


     cv::waitKey(0);
#endif

     return 1;
}
