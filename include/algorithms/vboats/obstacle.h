#ifndef VBOATS_OBSTACLE_H_
#define VBOATS_OBSTACLE_H_

#include <math.h>                  /* for atan2, cos, sin, sqrt */
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>

class Obstacle{
public:
    Obstacle(std::vector<cv::Point> pts, std::vector<int> dBounds) : minXY(pts[0]), maxXY(pts[1]), dMin(dBounds[0]), dMax(dBounds[1]){};
    int dMin;
    int dMax;
    cv::Point maxXY;
    cv::Point minXY;
    double angle;
    float distance;
    cv::Point3f location;
    void update(bool depth_based, float cam_baseline = 0, float cam_dscale = 0,
         float* cam_focal = nullptr, float* cam_principal_point = nullptr,
         float dtype_gain = 0, float aux_dist_factor = 0, bool verbose = true
    ){
          /** Default params for (Intel D415):
             --------------------------------
             focal     - 596.39, 596.39
             principle - 423.74, 242.01 (for 848 x 480)
             dscale    - 0.001
             baseline  - 0.014732
          */
          float fx = 596.39, fy = 596.39;
          if(cam_focal){
               fx = (float)cam_focal[0];
               fy = (float)cam_focal[1];
          }
          float ppx = 423.74, ppy = 242.01;
          if(cam_principal_point){
               ppx = (float)cam_principal_point[0];
               ppy = (float)cam_principal_point[1];
          }
          float dscale = 0.001;
          float baseline = 0.014732;
          if(cam_dscale != 0) dscale = cam_dscale;
          if(cam_baseline != 0) baseline = cam_baseline;

          float cvt_gain = 1.0;
          float aux_gain = 1.0;
          // float aux_gain = 1.35;
          if(dtype_gain != 0) cvt_gain = dtype_gain;
          if(aux_dist_factor != 0) aux_gain = aux_dist_factor;
          /** TODO: use this for more accurate  pixel information
          nonzero = umap.nonzero()
          nonzeroy = np.array(nonzero[0])
          nonzerox = np.array(nonzero[1])
          ymean = np.mean(ys)
          xmean = np.int(np.mean(nonzerox[good_inds]))
          dmean = np.mean(nonzeroy[good_inds])
          */
          float xmin = (float)this->minXY.x;
          float ymin = (float)this->minXY.y;
          float xmax = (float)this->maxXY.x;
          float ymax = (float)this->maxXY.y;
          float dmin = (float)this->dMin;
          float dmax = (float)this->dMax;

          float xmean = (xmin + xmax) / 2.0;
          float ymean = (ymin + ymax) / 2.0;
          float dmean = (dmin + dmax) / 2.0;

          float zgain, pz;
          if(depth_based){
               zgain = (65535.0 / 255.0)*dscale;
               pz = dmean * zgain;
          } else{
               float tmpDmean = dmean / cvt_gain;
               zgain = fx*baseline;
               pz = zgain / tmpDmean;
          }

          float x = ((xmean - ppx) * pz) / fx;
          float y = ((ymean - ppy) * pz) / fy;
          float dist = std::sqrt(x*x + pz*pz);
          double theta = std::atan2(x,pz);
          if(verbose) printf("Obstacle Relative Distance =  %.2fm --- Angle = %.2lf deg --- Position (X,Y,Z) = (%.2f, %.2f, %.2f) m \r\n", dist, theta*M_RAD2DEG, x,y, dmean);
          this->location.x = x;
          this->location.y = y;
          this->location.z = pz;
          this->distance = dist;
          this->angle = theta;
     }
};

#endif // VBOATS_OBSTACLE_H_
