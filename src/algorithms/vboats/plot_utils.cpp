#include <iostream>
#include <opencv2/opencv.hpp>
#include "algorithms/vboats/plot_utils.h"


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
