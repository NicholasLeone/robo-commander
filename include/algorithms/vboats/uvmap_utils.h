#ifndef VBOATS_UVMAP_UTILS_H_
#define VBOATS_UVMAP_UTILS_H_

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

using namespace std;

void genUVMap(cv::Mat image, cv::Mat* umap, cv::Mat* vmap, bool verbose = false);
void genUVMapThreaded(cv::Mat image, cv::Mat* umap, cv::Mat* vmap, double nThreads = -1.0, bool verbose = false);

/** WARNING: Experimental function */
void genUVMapScaled(cv::Mat image, cv::Mat* umap, cv::Mat* vmap, double scale, bool verbose = false);

/** Visualization Helpers */
void displayUVMaps(const cv::Mat& umap, const cv::Mat& vmap, bool colorize = false);
void displayUVMapOverlay(const cv::Mat& image, const cv::Mat& umap, const cv::Mat& vmap);

#endif // VBOATS_UVMAP_UTILS_H_
