#ifndef SRC_MAPFRAME_H
#define SRC_MAPFRAME_H

#include "opencv2/core/core.hpp"
#include "ApplicationState.h"
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace std;
using namespace cv;

class MapFrame {
public:
    int SUB_W = 0;
    int SUB_H = 0;
    int PATCH_WIDTH = 0;
    int PATCH_HEIGHT = 0;

    Mat regionOutput;
    Mat patchData;

    void update(ApplicationState app);
    void setRegionOutput(Mat &regionOutput);

    void setPatchData(Mat &patchData);
    Mat& getRegionOutput();
    Mat& getPatchData();

     void drawGraph(Mat& img);
};


#endif //SRC_MAPFRAME_H
