#ifndef SRC_MAPFRAMEPROCESSOR_H
#define SRC_MAPFRAMEPROCESSOR_H


#include "MapFrame.h"
#include "PlanarRegion.h"
#include <opencv2/highgui.hpp>

class MapFrameProcessor {
public:

    // MapFrame* frame;
    bool visited[60][80] = {0};
    int region[60][80] = {0};

    void generateSegmentation(MapFrame frame, vector<shared_ptr<PlanarRegion>>& planarRegionList);
    void dfs(int x, int y, int component, int& num, Mat& debug, uint8_t* framePatch, shared_ptr<PlanarRegion> planarRegion, MapFrame inputFrame);

    int adx[8] = {-1, 0, 1, 1, 1, 0, -1, -1};
    int ady[8] = {-1, -1, -1, 0, 1, 1, 1, 0};

};


#endif //SRC_MAPFRAMEPROCESSOR_H
