#include "utilities/utils.h"
#include "utilities/image_utils.h"

#include "algorithms/vboats/uvmap_utils.h"

using namespace std;

void genUVMap(cv::Mat image, cv::Mat* umap, cv::Mat* vmap, bool verbose){
     double minVal, maxVal;
     cv::minMaxLoc(image, &minVal, &maxVal);
     int dmax = (int) maxVal + 1;
     if(verbose) printf("dmax = %d, min, max = %.2f, %.2f, h = %d, w = %d\r\n",dmax,minVal, maxVal,image.rows,image.cols);

     cv::Mat umapMat = cv::Mat::zeros(dmax, image.cols, CV_8UC1);
     /** Size Temporarily transposed for ease of for-loop copying (Target size = h x dmax)*/
     cv::Mat vmapMat = cv::Mat::zeros(dmax, image.rows, CV_8UC1);

     int channels[] = {0};
     int histSize[] = {dmax};
     float sranges[] = { 0, dmax };
     const float* ranges[] = { sranges };

     /** Non-parallized Umap */
     cv::MatND histU;
	for(int i = 0; i < image.cols; i++){
		cv::Mat uscan = image.col(i);
		cv::calcHist(&uscan, 1, channels, cv::Mat(), histU, 1, histSize, ranges);
		histU.col(0).copyTo(umapMat.col(i));
	}
     /** Non-parallized Vmap */
     cv::MatND histV;
	for(int j = 0; j < image.rows; j++){
          cv::Mat vscan = image.row(j);
          cv::calcHist(&vscan, 1, channels, cv::Mat(), histV, 1, histSize, ranges);
          histV.col(0).copyTo(vmapMat.col(j));
	}
     /** Correct generated vmap's dimensions */
     vmapMat = vmapMat.t();

     /** EXPERIMENTAL:
     cv::Mat _umap, _vmap;
     if(dmax != 256){
          printf("Adding uvmap buffers\r\n");
          // yoffset = 256 - dmax;
          // int xoffset = 0, yoffset = 0;
          int offset = 256 - dmax;
          cv::Mat umapBuf = cv::Mat::zeros(offset, image.cols, CV_8UC1);
     	cv::Mat vmapBuf = cv::Mat::zeros(image.rows, offset, CV_8UC1);
          // cv::vconcat(umapBuf, umapMat, _umap);
          // cv::hconcat(vmapBuf, vmapMat, _vmap);
          cv::vconcat(umapMat, umapBuf, _umap);
          cv::hconcat(vmapMat, vmapBuf, _vmap);
     } else{
          _umap = umapMat;
          _vmap = vmapMat;
     }
     */

     if(umap) *umap = umapMat;
     if(vmap) *vmap = vmapMat;
}

void genUVMapThreaded(cv::Mat image, cv::Mat* umap, cv::Mat* vmap, double nThreads, bool verbose){
     double minVal, maxVal;
     cv::minMaxLoc(image, &minVal, &maxVal);
     int dmax = (int) maxVal + 1;

     cv::Mat umapMat = cv::Mat::zeros(dmax, image.cols, CV_8UC1);
     /** Size Temporarily transposed for ease of for-loop copying (Target size = h x dmax)*/
     cv::Mat vmapMat = cv::Mat::zeros(dmax, image.rows, CV_8UC1);

     int channels[] = {0};
     int histSize[] = {dmax};
     float sranges[] = { 0, dmax };
     const float* ranges[] = { sranges };

     /** Parallized Umap */
     cv::parallel_for_(cv::Range(0,image.cols), [&](const cv::Range& range){
          cv::MatND histU;
          for(int i = range.start; i < range.end; i++){
               cv::Mat uscan = image.col(i);
               cv::calcHist(&uscan, 1, channels, cv::Mat(), histU, 1, histSize, ranges);
               histU.col(0).copyTo(umapMat.col(i));
          }
     },nThreads);

     /** Parallized Vmap */
     cv::parallel_for_(cv::Range(0, image.rows), [&](const cv::Range& range){
          cv::MatND histV;
          for(int j = range.start; j < range.end; j++){
               cv::Mat vscan = image.row(j);
               cv::calcHist(&vscan, 1, channels, cv::Mat(), histV, 1, histSize, ranges);
               histV.col(0).copyTo(vmapMat.col(j));
          }
     },nThreads);
     /** Correct generated vmap's dimensions */
     vmapMat = vmapMat.t();
     // cv::Mat vmapTrans = vmapMat.t();

     /** EXPERIMENTAL:
     cv::Mat _umap, _vmap;
     if(dmax != 256){
          printf("Adding uvmap buffers\r\n");
          // yoffset = 256 - dmax;
          // int xoffset = 0, yoffset = 0;
          int offset = 256 - dmax;
          cv::Mat umapBuf = cv::Mat::zeros(offset, image.cols, CV_8UC1);
     	cv::Mat vmapBuf = cv::Mat::zeros(image.rows, offset, CV_8UC1);
          // cv::vconcat(umapBuf, umapMat, _umap);
          // cv::hconcat(vmapBuf, vmapMat, _vmap);
          cv::vconcat(umapMat, umapBuf, _umap);
          cv::hconcat(vmapMat, vmapBuf, _vmap);
     } else{
          _umap = umapMat;
          _vmap = vmapMat;
     }
     */

     if(umap) *umap = umapMat;
     if(vmap) *vmap = vmapMat;
}

void genUVMapScaled(cv::Mat image, cv::Mat* umap, cv::Mat* vmap, double scale, bool verbose){
     int w = image.cols;
     int h = image.rows;
     double minVal, maxVal;
     // cv::Mat scaledImage = cv::Mat_<uchar>(image*scale);
     cv::Mat scaledImage = image;
     cv::minMaxLoc(scaledImage, &minVal, &maxVal);
     if(verbose) printf("min, max = %.2f, %.2f, h = %d, w = %d\r\n",minVal, maxVal,h,w);
     int dmax = (int) maxVal + 1;

	int hu = dmax; int wu = w; int hv = h; int wv = dmax;
	// cv::Mat _umap = cv::Mat::zeros(dmax, w, image.type());
	// cv::Mat _vmap = cv::Mat::zeros(h, dmax, image.type());
	cv::Mat _umap = cv::Mat::zeros(dmax, w, CV_8UC1);
	cv::Mat _vmap = cv::Mat::zeros(h, dmax, CV_8UC1);

	// cv::Mat _umapNorm = cv::Mat::zeros(dmax, w, CV_8UC1);
	// cv::Mat _vmapNorm = cv::Mat::zeros(h, dmax, CV_8UC1);
     if(verbose) printf("%s\r\n",cvStrSize("Initialized Umap",_umap).c_str());

     int channels[] = {0};
     int histSize[] = {dmax};
	float sranges[] = { 0, dmax };
	const float* ranges[] = { sranges };
     #pragma omp parallel for
	for(int i = 0; i < w; i++){
          cv::MatND histU;
		cv::Mat uscan = scaledImage.col(i);
		cv::calcHist(&uscan, 1, channels, cv::Mat(), histU, 1, histSize, ranges);
		histU.col(0).copyTo(_umap.col(i));
	}

     #pragma omp parallel for
	for(int i = 0; i < h; i++){
		cv::MatND hist;
		cv::Mat vscan = scaledImage.row(i);
		cv::calcHist(&vscan, 1, channels, cv::Mat(), hist, 1, histSize, ranges);
		cv::Mat histrow = hist.t();
		histrow.row(0).copyTo(_vmap.row(i));
	}

     cv::Mat _umap8, _vmap8;
     cv::Mat spreadUmap, spreadVmap;
     if(maxVal < 255.0){
          double spreadRatio = 256.0 / (maxVal+1.0);
          printf("spread ratio = %.4f\r\n",spreadRatio);
          spread_image(_umap, &spreadUmap, 1.0, spreadRatio, nullptr);
          spread_image(_vmap, &spreadVmap, spreadRatio, 1.0, nullptr);
          cvinfo(_umap,"raw umap");
          cvinfo(_vmap,"raw vmap");
          cvinfo(spreadUmap,"Spread umap");
          cvinfo(spreadVmap,"Spread vmap");
          cv::imshow("raw Umap", _umap);
          cv::imshow("raw Vmap", _vmap);
          cv::imshow("spread umap", spreadUmap);
          cv::imshow("spread vmap", spreadVmap);
          cv::waitKey(0);
     } else{
          spreadUmap = _umap.clone();
          spreadVmap = _vmap.clone();
     }

     if(scaledImage.type() != CV_8UC1){
          printf("UVMap Generator --- Converting uv map data type to uint8...\r\n");
          _umap.convertTo(_umap8, CV_8UC1, (255.0/65535.0));
          _vmap.convertTo(_vmap8, CV_8UC1, (255.0/65535.0));
          *umap = _umap8;
          *vmap = _vmap8;
     }else{
          *umap = _umap;
          *vmap = _vmap;
     }
}


/** Visualization Helpers */
void displayUVMaps(const cv::Mat& umap, const cv::Mat& vmap, char* title, bool colorize, bool autoSize){
     if(umap.empty()){
          printf("[WARN] displayUVMaps() --- Input umap image is empty, not generating display image.\r\n");
          return;
     }
     if(vmap.empty()){
          printf("[WARN] displayUVMaps() --- Input vmap image is empty, not generating display image.\r\n");
          return;
     }
     cv::Mat dispU, dispV;
     std::string lblU, lblV, lblTitle;
     if(!title) lblTitle = "Title";
     else lblTitle = title;
     if(colorize){
          /** Check uv maps for proper data type before colorizing */
          cv::Mat umap8, vmap8;
          double cvtGain = (255.0/65535.0);
          /** Umap */
          if(umap.type() != CV_8UC1) umap.convertTo(umap8, CV_8UC1, cvtGain);
          else umap8 = umap;
          /** Vmap */
          if(vmap.type() != CV_8UC1) vmap.convertTo(vmap8, CV_8UC1, cvtGain);
          else vmap8 = vmap;
          /** Colorize */
          cv::applyColorMap(umap8, dispU, cv::COLORMAP_JET);
          cv::applyColorMap(vmap8, dispV, cv::COLORMAP_JET);

          lblU = format("UMap[CV_8UC1] %s",lblTitle.c_str());
          lblV = format("VMap[CV_8UC1] %s",lblTitle.c_str());
     } else{
          dispU = umap; dispV = vmap;
          lblU = format("UMap[%s] %s", cvtype2str(dispU.type()).c_str(),lblTitle.c_str());
          lblV = format("VMap[%s] %s", cvtype2str(dispV.type()).c_str(),lblTitle.c_str());
     }

     if(autoSize){
          cv::namedWindow(lblU.c_str(), cv::WINDOW_AUTOSIZE);
          cv::namedWindow(lblV.c_str(), cv::WINDOW_AUTOSIZE);
     } else{
          cv::namedWindow(lblU.c_str(), cv::WINDOW_NORMAL);
          cv::namedWindow(lblV.c_str(), cv::WINDOW_NORMAL);
     }

     cv::imshow(lblU, dispU);
     cv::imshow(lblV, dispV);
}
void displayUVMapOverlay(const cv::Mat& image, const cv::Mat& umap, const cv::Mat& vmap){}
