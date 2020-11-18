#ifndef SRC_MAPFRAME_H
#define SRC_MAPFRAME_H

#include "opencv2/core/core.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace std;
using namespace cv;

class MapFrame {
public:
    void setRegionOutput(Mat &regionOutput);
    void setPatchData(Mat &patchData);

     Mat& getRegionOutput();
     Mat& getPatchData();
     void drawGraph(Mat& img);

public:
    const int SUB_W = 80;
    const int SUB_H = 60;
    Mat regionOutput = Mat(SUB_H, SUB_W, CV_32FC(6));
    Mat patchData = Mat(SUB_H, SUB_W, CV_8UC1);
};


#endif //SRC_MAPFRAME_H
