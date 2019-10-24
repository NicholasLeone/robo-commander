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

void filter_disparity_vmap(const cv::Mat& input, cv::Mat* output, bool verbose){
     static const std::vector<float> thresholds = {0.85,0.85,0.75,0.5};
     cv::Mat filtered;
     std::vector<cv::Mat> strips;
     int nThreshs = thresholds.size();

     int h = input.rows;
     int w = input.cols;
     int c = input.channels();

     double minVal, maxVal;
     cv::minMaxLoc(input, &minVal, &maxVal);

     int nStrips = int(ceil(maxVal/256.0) * nThreshs);
     int vMax = (int)maxVal;
     printf("vMax, nThreshs: %d, %d\r\n", vMax, nStrips);

     int nSections = 3;
     int dSections = int(h/float(nSections));

     // topV = raw_vmap[0:dh, :]
     // botV = raw_vmap[dh:h, :]
     // tmpThresh = int(np.max(topV)*0.05)
     // _,topV = cv2.threshold(topV, tmpThresh,255,cv2.THRESH_TOZERO)
     // preVmap = np.concatenate((topV,botV), axis=0)

     /**
     stripsPV = []
     stripsV = strip_image(preVmap, nstrips=nThreshs, horizontal_strips=False)
     for i, strip in enumerate(stripsV):
         tmpMax = np.max(strip)
         tmpMean = np.mean(strip)
         tmpStd = np.std(strip)
         if(tmpMean == 0):
             stripsPV.append(strip)
             continue
         if(verbose): print("---------- [Strip %d] ---------- \r\n\tMax = %.1f, Mean = %.1f, Std = %.1f" % (i,tmpMax,tmpMean,tmpStd))
         dratio = vMax/255.0
         relRatio = (tmpMax-tmpStd)/float(vMax)
         rrelRatio = (tmpMean)/float(tmpMax)
         if(verbose): print("\tRatios: %.3f, %.3f, %.3f" % (dratio, relRatio, rrelRatio))
         if(relRatio >= 0.4): gain = relRatio + rrelRatio
         else: gain = 1.0 - (relRatio + rrelRatio)
         thresh = int(thresholds[i]* gain * tmpMax)
         if(verbose): print("\tGain = %.2f, Thresh = %d" % (gain,thresh))
         _, tmpStrip = cv2.threshold(strip, thresh,255,cv2.THRESH_TOZERO)
         stripsPV.append(tmpStrip)

     filtV = np.concatenate(stripsPV, axis=1)
     return filtV, stripsV, stripsPV
     */

}


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


     /**
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
     */

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
     // cv::Rect remainURoi = cv::Rect();
     // cv::Rect remainVRoi = cv::Rect();

     std::cout << "Umap ROI: " << mainURoi << std::endl;
     std::cout << "Vmap ROI: " << mainVRoi << std::endl;

     cv::Mat croppedUmap = umap(mainURoi);
     cv::Mat croppedVmap = vmap(mainVRoi);

     cv::rectangle(copyU, mainURoi, cv::Scalar(0, 0, 255), 1);
     cv::rectangle(copyV, mainVRoi, cv::Scalar(0, 0, 255), 1);
     cv::imshow("cropped region umap", copyU);
     cv::imshow("cropped region vmap", copyV);
     cv::waitKey(0);

     // alpha = 0.4;
     // image_new = cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0)

     cv::Size uSpreadTargetSz = cv::Size(umap.size());
     cv::Size vSpreadTargetSz = cv::Size(vmap.size());
     cv::Size uUnspreadTargetSz = cv::Size(croppedUmap.size());
     cv::Size vUnspreadTargetSz = cv::Size(croppedVmap.size());

     cv::Mat dispU, dispV;
     cv::cvtColor(croppedUmap, dispU, cv::COLOR_GRAY2BGR);
     cv::cvtColor(croppedVmap, dispV, cv::COLOR_GRAY2BGR);
     // cv::applyColorMap(croppedUmap, dispU, cv::COLORMAP_JET);
     // cv::applyColorMap(croppedVmap, dispV, cv::COLORMAP_JET);

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

     // for(cv::Mat strip : strips){
     //      cv::imshow("strip", strip);
     //      cv::waitKey(0);
     // }
     // for(cv::Mat strip : stripsvert){
     //      cv::imshow("strip", strip);
     //      cv::waitKey(0);
     // }

     // cv::cvtColor(unspreadUmap, unspreadUmap, cv::COLOR_BGR2GRAY);
     // cv::cvtColor(unspreadUmap, unspreadUmap, cv::COLOR_BGR2GRAY);
     // cv::Mat merged = cv::Mat::zeros(unspreadUmap.size(),unspreadUmap.type());
     // trunctaed.copyTo(target(cv::Rect(0, 0, 30, 30)));

     cv::Mat resultU = cv::Mat::zeros(unspreadUmap.size(),unspreadUmap.type());
     cv::Mat resultV = cv::Mat::zeros(unspreadVmap.size(),unspreadVmap.type());
     origU.copyTo(resultU, unspreadUmap);
     origV.copyTo(resultV, unspreadVmap);

     printf("%s\r\n",cvStrSize("Result Umap",resultU).c_str());
     printf("%s\r\n",cvStrSize("Result Vmap",resultV).c_str());
     cv::namedWindow("result umap", cv::WINDOW_NORMAL );
     cv::namedWindow("result vmap", cv::WINDOW_NORMAL );
     cv::imshow("result umap", resultU);
     cv::imshow("result vmap", resultV);
     cv::waitKey(0);


     return 1;
}
