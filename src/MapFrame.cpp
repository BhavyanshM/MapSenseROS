
#include "MapFrame.h"

void MapFrame::update(ApplicationState app){
    this->SUB_W = app.SUB_W;
    this->SUB_H = app.SUB_H;
    this->PATCH_HEIGHT = app.PATCH_HEIGHT;
    this->PATCH_WIDTH = app.PATCH_WIDTH;
}

void MapFrame::setRegionOutput(Mat &regionOutput) {
   this->regionOutput = regionOutput;
}

void MapFrame::setPatchData(Mat &patchData) {
    this->patchData = patchData;
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
                line(img, Point(i*PATCH_HEIGHT + PATCH_HEIGHT/2, j*PATCH_WIDTH + PATCH_WIDTH/2), Point((i+1)*PATCH_HEIGHT + PATCH_HEIGHT/2, j*PATCH_WIDTH + PATCH_WIDTH/2), Scalar(0,150,0), 1);
                line(img, Point(i*PATCH_HEIGHT + PATCH_HEIGHT/2, j*PATCH_WIDTH + PATCH_WIDTH/2), Point(i*PATCH_HEIGHT + PATCH_HEIGHT/2, (j+1)*PATCH_WIDTH + PATCH_WIDTH/2), Scalar(0, 150,0), 1);
                circle(img, Point(i*PATCH_HEIGHT + PATCH_HEIGHT/2, j*PATCH_WIDTH + PATCH_WIDTH/2), 2, Scalar(0,200,0), -1);
            }
        }
    }
}

