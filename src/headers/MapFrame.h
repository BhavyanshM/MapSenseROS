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

    Mat regionOutput;
    Mat patchData;

    void setRegionOutput(Mat &regionOutput);

    void setPatchData(Mat &patchData);
    Mat& getRegionOutput();
    Mat& getPatchData();

    void drawGraph(Mat& img, ApplicationState app);
    Vec6f getPatch(int x, int y);
};


#endif //SRC_MAPFRAME_H
