#ifndef DARKNET_ALG_H_
#define DARKNET_ALG_H_
#include <iostream>
#include <vector>
#include <string> 
#include "darknet.h"
#include <cv_bridge/cv_bridge.h>

using namespace std;

// 用于存储检测结果的结构
struct detection_with_class {
    detection det;
    int best_class;
};

vector<detection_with_class> detect(image im, network *net, float thresh, float hier_thresh, float nms, int classes);
image getCvImage(cv::Mat rgbImage);
cv::Mat imageToMat(image img);
#endif

