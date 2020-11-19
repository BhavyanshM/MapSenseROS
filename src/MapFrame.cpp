
#include "MapFrame.h"

MapFrame::MapFrame(){
    this->regionOutput = Mat(SUB_H, SUB_W, CV_32FC(6));
    this->patchData = Mat(SUB_H, SUB_W, CV_8UC1);
}

void MapFrame::setRegionOutput(Mat &regionOutput) {
    MapFrame::regionOutput = regionOutput;
}

void MapFrame::setPatchData(Mat &patchData) {
    MapFrame::patchData = patchData;
}

Mat& MapFrame::getRegionOutput() {
    return regionOutput;
}

Mat& MapFrame::getPatchData() {
    return patchData;
}

void MapFrame::drawGraph(Mat& img){
    for(int j = 0; j<SUB_H-1; j++){
        for(int i = 0; i<SUB_W-1; i++){
            if (patchData.at<uint8_t>(j,i) == 255){
                line(img, Point(i*8 + 4, j*8 + 4), Point((i+1)*8 + 4, j*8 + 4), Scalar(0,150,0), 1);
                line(img, Point(i*8 + 4, j*8 + 4), Point(i*8 + 4, (j+1)*8 + 4), Scalar(0, 150,0), 1);
                circle(img, Point(i*8 + 4, j*8 + 4), 2, Scalar(0,200,0), -1);
            }
        }
    }
}

