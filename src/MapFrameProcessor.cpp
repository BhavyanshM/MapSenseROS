
#include "MapFrameProcessor.h"

void MapFrameProcessor::generateSegmentation(MapFrame inputFrame, vector<shared_ptr<PlanarRegion>>& planarRegionList) {
    int components = 0;
    vector<int> sizes;
    vector<int> ids;

    this->frame = &inputFrame;
    uint8_t* framePatch = reinterpret_cast<uint8_t *>(frame->patchData.data);
    Mat debug(600,800, CV_8UC3);
    debug = Scalar(0);
    for (int i = 0; i < frame->SUB_H; i++) {
        for (int j = 0; j < frame->SUB_W; j++) {
            uint8_t patch = framePatch[i*80 + j];
            if (!visited[i][j] && patch == 255) {
                int num = 0;
                shared_ptr<PlanarRegion> planarRegion = make_shared<PlanarRegion>(components);
                dfs(i, j, components, num, debug, framePatch, planarRegion);
                if (num > 20 && num - planarRegion->getNumOfVertices() > 20){
                    planarRegionList.emplace_back(planarRegion);
                    components++;
                    sizes.push_back(num);
                    ids.push_back(components);
                }
            }
        }
    }
}


void MapFrameProcessor::dfs(int x, int y, int component, int& num, Mat& debug, uint8_t* framePatch, shared_ptr<PlanarRegion> planarRegion) {
    if (visited[x][y]) return;

    num++;
    visited[x][y] = true;
    region[x][y] = component;
    Vec6f patch = this->frame->getRegionOutput().at<Vec6f>(x,y);
    planarRegion->addPatch(Vector3f(patch[0], patch[1], patch[2]), Vector3f(patch[3], patch[4], patch[5]));
    // circle(debug, Point((y)*10, (x)*10), 2, Scalar((component+1)*231 % 255,(component+1)*123 % 255,(component+1)*312 % 255), -1);

    int count = 0;
    for (int i = 0; i < 8; i++) {
        if (x + adx[i] < frame->SUB_H - 1 && x + adx[i] > 1 && y + ady[i] < frame->SUB_W - 1 && y + ady[i] > 1) {
            uint8_t newPatch = framePatch[ (x + adx[i])*80 + (y + ady[i])];
            if (newPatch == 255){
                count++;
                dfs(x+adx[i], y+ady[i], component, num, debug, framePatch, planarRegion);
            }
        }
    }
    if (count != 8) {
        planarRegion->insertVertex(Vector3f(patch[3], patch[4], patch[5]));
        circle(debug, Point((y)*10, (x)*10), 2, Scalar(255,255,255), -1);
    }

    imshow("Debug", debug);
    if(waitKeyEx(1) == 1048689) exit(1);
}

// PlanarRegionPolygonizer
// PlanarRegionSegmentationMessage
//