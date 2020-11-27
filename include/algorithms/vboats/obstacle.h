#ifndef VBOATS_OBSTACLE_H_
#define VBOATS_OBSTACLE_H_

#include <vector>
#include <opencv2/core/types.hpp>

class Obstacle{
public:
    Obstacle(std::vector<cv::Point> pts, std::vector<int> dBounds);

    void update(bool depth_based,
         float cam_baseline = 0, float cam_dscale = 0,
         std::vector<float> cam_focal = {},
         std::vector<float> cam_principal_point = {},
         float dtype_gain = 0,
         float aux_dist_factor = 0,
         bool verbose = true
    );

    cv::Point3f get_location();
    std::string toString();

public:
    int dMin;
    int dMax;
    cv::Point maxXY;
    cv::Point minXY;
    double _angle;
    float _distance;
    cv::Point3f _location;

};

#endif // VBOATS_OBSTACLE_H_
