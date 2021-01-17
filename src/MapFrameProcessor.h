#ifndef SRC_MAPFRAMEPROCESSOR_H
#define SRC_MAPFRAMEPROCESSOR_H


#include "MapFrame.h"
#include "PlanarRegion.h"
#include <opencv2/highgui.hpp>
#include "ApplicationState.h"
#include <algorithm>

class MapFrameProcessor {
public:

    // MapFrame* frame;
    Mat debug = Mat(600,800, CV_8UC3);
    bool visited[60][80] = {0};
    bool boundary[60][80] = {0};
    int region[60][80] = {0};
    ApplicationState params;

    void generateSegmentation(MapFrame frame, vector<shared_ptr<PlanarRegion>>& planarRegionList, ApplicationState params);
    void dfs(int x, int y, int component, int& num, Mat& debug, uint8_t* framePatch, shared_ptr<PlanarRegion> planarRegion, MapFrame inputFrame);
    void boundary_dfs(int x, int y, int component, int& num, Mat& debug, shared_ptr<RegionHole> planarRegion, MapFrame inputFrame);
    void findBoundaryAndHoles(MapFrame frame, vector<shared_ptr<PlanarRegion>>& planarRegionList);
    void printPatchGraph(MapFrame inputFrame);

    int adx[8] = {-1, 0, 1, 1, 1, 0, -1, -1};
    int ady[8] = {-1, -1, -1, 0, 1, 1, 1, 0};

};


#endif //SRC_MAPFRAMEPROCESSOR_H
