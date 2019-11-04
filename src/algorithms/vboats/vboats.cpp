#include "algorithms/vboats/vboats.h"
#include "algorithms/vboats/image_utils.h"

#include <chrono>

using namespace std;
using namespace chrono;


VBOATS::VBOATS(){}
VBOATS::~VBOATS(){}

void VBOATS::init(){}
void VBOATS::update(){}


float VBOATS::get_uv_map(cv::Mat image, cv::Mat* umap, cv::Mat* vmap,
     bool visualize, std::string dispId, bool verbose, bool debug, bool timing)
{
     float dt = -1.0;
     high_resolution_clock::time_point prev_time, now;
     if(timing) prev_time = high_resolution_clock::now();

     int w = image.cols;
     int h = image.rows;
     double minVal, maxVal;
     cv::minMaxLoc(image, &minVal, &maxVal);
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
		cv::MatND hist, histNorm;
		cv::Mat uscan = image.col(i);
		cv::calcHist(&uscan, 1, channels, cv::Mat(), hist, 1, histSize, ranges);
          // cv::normalize(hist, histNorm, 0, 255, cv::NORM_MINMAX, CV_8UC1);
		if(debug){
               printf("Umap Column %d: h = %d, w = %d\r\n",i, hist.rows, hist.cols);
               // printf("Normalized Umap Column %d: h = %d, w = %d\r\n",i, histNorm.rows, histNorm.cols);
          }
		hist.col(0).copyTo(_umap.col(i));
		// histNorm.col(0).copyTo(_umapNorm.col(i));
	}
     // printf("%s\r\n",cvStrSize("Genereated Umap",_umap).c_str());
     #pragma omp parallel for
	for(int i = 0; i < h; i++){
		cv::MatND hist;
          cv::Mat histNorm;
		cv::Mat vscan = image.row(i);
		cv::calcHist(&vscan, 1, channels, cv::Mat(), hist, 1, histSize, ranges);
		cv::Mat histrow = hist.t();
          // cv::normalize(histrow, histNorm, 0, 255, cv::NORM_MINMAX, CV_8UC1);
		if(debug){
               printf("VMap Row %d: h = %d, w = %d\r\n",i, histrow.rows, histrow.cols);
               // printf("Normalized Vmap Row %d: h = %d, w = %d\r\n",i, histNorm.rows, histNorm.cols);
          }
		histrow.row(0).copyTo(_vmap.row(i));
		// histNorm.row(0).copyTo(_vmapNorm.row(i));
	}
     // printf("%s\r\n",cvStrSize("Genereated Vmap",_vmap).c_str());

     cv::Mat _umap8, _vmap8;
     if(visualize){
          std::string strU = format("UMap[%s] %s", cvtype2str(_umap.type()).c_str(), dispId.c_str());
          std::string strV = format("VMap[%s] %s", cvtype2str(_vmap.type()).c_str(), dispId.c_str());
          std::string strU8 = format("UMap[CV_8UC1] %s", dispId.c_str());
          std::string strV8 = format("VMap[CV_8UC1] %s", dispId.c_str());
          cv::namedWindow(strU.c_str(), cv::WINDOW_NORMAL );
          cv::namedWindow(strV.c_str(), cv::WINDOW_NORMAL );
          cv::imshow(strU, _umap);
          cv::imshow(strV, _vmap);
          if(image.type() != CV_8UC1){
               cv::namedWindow(strU8.c_str(), cv::WINDOW_NORMAL );
               cv::namedWindow(strV8.c_str(), cv::WINDOW_NORMAL );
               _umap.convertTo(_umap8, CV_8UC1, (255.0/65535.0));
               _vmap.convertTo(_vmap8, CV_8UC1, (255.0/65535.0));
               cv::imshow(strU8, _umap8);
               cv::imshow(strV8, _vmap8);
          }
     }
     if(image.type() != CV_8UC1){
          printf("UVMap Generator --- Converting uv map data type to uint8...\r\n");
          _umap.convertTo(_umap8, CV_8UC1, (255.0/65535.0));
          _vmap.convertTo(_vmap8, CV_8UC1, (255.0/65535.0));
          *umap = _umap8;
          *vmap = _vmap8;
     }else{
          *umap = _umap;
          *vmap = _vmap;
     }

     if(timing){
          // duration<float> time_span = duration_cast<duration<float>>(now - _prev_time);
          // dt = time_span.count();
          now = high_resolution_clock::now();
          dt = duration_cast<duration<float>>(now - prev_time).count();
     }
     return dt;
}



/**
def update(self):
     nObs = 0
     lineParams = None
     img = np.copy(self.disparity)
     raw_umap, raw_vmap, _ = self.vboat.get_uv_map(img)
     cv2.rectangle(raw_vmap,(0,0),(3, raw_vmap.shape[0]),(0,0,0), cv2.FILLED)
     cv2.rectangle(raw_umap,(0,0),(raw_umap.shape[1], 3),(0,0,0), cv2.FILLED)
     self.umap = np.copy(raw_umap)
     self.vmap = np.copy(raw_vmap)

     vmapFiltered,_,_ = self.vboat.filter_disparity_umap(raw_vmap,[0.25,0.15,0.35,0.35])
     vmapLessFiltered,_,_ = self.vboat.filter_disparity_vmap(raw_vmap,[0.15,0.15,0.01,0.01])

     validGnd,bestM,bestIntercept, sets = self.vboat.estimate_houghline_coeffs(vmapFiltered)
     if validGnd: lineParams = [bestM,bestIntercept]

     filtU, strips, stripsT = self.vboat.filter_disparity_umap(raw_umap,[0.15,0.15,0.25,0.35])
     kernelI = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
     filtU = cv2.morphologyEx(filtU, cv2.MORPH_CLOSE, kernelI)
     filtered_contours,contours = self.vboat.find_contours(filtU,threshold = 100.0, max_thresh=-1)

     xLims, dLims, _ = self.vboat.extract_contour_bounds(filtered_contours)
     obs, obsU, ybounds, dbounds, windows, nObs = self.vboat.find_obstacles_disparity(vmapLessFiltered, dLims, xLims, ground_detected=validGnd, lineCoeffs = lineParams, verbose=False)
     # print(nObs)

     distances, angles = self.vboat.extract_obstacle_information_disparity(raw_umap, xLims, dbounds, obs,focal=self.focal,baseline=self.baseline,dscale=self.cam.dscale, pp=self.ppoint, dtype_cvt_gain=self.disparity2uintGain)

   if(self.flag_show_imgs):
       cpyV = np.copy(vmapLessFiltered)
       dispWindows = cv2.applyColorMap(cpyV,cv2.COLORMAP_PARULA)

       for wins in windows:
           for win in wins:
               cv2.rectangle(dispWindows,win[0],win[1],(0,255,255), 1)
       if validGnd:
           w = cpyV.shape[1]
           tmpy = int(w * bestM + bestIntercept)
           cv2.line(dispWindows,(0,bestIntercept), (w,tmpy), (0,0,255),1)

       try:
           overlay = make_uv_overlay(cv2.cvtColor(self.disparity,cv2.COLOR_GRAY2BGR),self.umap,self.vmap)
           cv2.imshow('overlay', overlay)
       except:
           print("[WARNING] Failed to create Overlay")
           pass
       cv2.imshow('disparity', dispWindows)
        return nObs, obs
*/
