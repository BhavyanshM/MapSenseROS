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
    Mat debug;
    MatrixXi visited;
    MatrixXi boundary;
    MatrixXi region;
    ApplicationState params;

    void init(ApplicationState app);
    void generateSegmentation(MapFrame frame, vector<shared_ptr<PlanarRegion>>& planarRegionList, ApplicationState params);
    void dfs(int x, int y, int component, int& num, Mat& debug, uint8_t* framePatch, shared_ptr<PlanarRegion> planarRegion, MapFrame inputFrame);
    void boundary_dfs(int x, int y, int component, int& num, Mat& debug, shared_ptr<RegionRing> regionRing, MapFrame inputFrame);
    void findBoundaryAndHoles(MapFrame frame, vector<shared_ptr<PlanarRegion>>& planarRegionList);
    void printPatchGraph(MapFrame inputFrame);

    int adx[8] = {-1, 0, 1, 1, 1, 0, -1, -1};
    int ady[8] = {-1, -1, -1, 0, 1, 1, 1, 0};

};


#endif //SRC_MAPFRAMEPROCESSOR_H
