#include "MapFrameProcessor.h"


void printPatchGraph(MapFrame inputFrame){
    for(int i = 0; i<inputFrame.SUB_H; i++){
        for(int j = 0; j<inputFrame.SUB_W; j++){
            uint8_t current = inputFrame.patchData.at<uint8_t>(i,j);
            if (current == 255){
                printf("1 ");
            }else{
                printf("0 ");
            }
        }
        printf("\n");
    }
}

void MapFrameProcessor::generateSegmentation(MapFrame inputFrame, vector<shared_ptr<PlanarRegion>>& planarRegionList, ApplicationState params) {
    printf("Starting DFS for Segmentation\n");

    /* For initial development only. Delete all old previous regions before inserting new ones. */
    for(int i = 0; i<planarRegionList.size(); i++){
        planarRegionList[i].reset();
    }
    planarRegionList.clear();

    int components = 0;
    vector<int> sizes;
    vector<int> ids;

    memset(visited, 0, sizeof(bool) * 60 * 80);
    memset(region, 0, sizeof(int) * 60 * 80);

    // printPatchGraph(inputFrame);

    uint8_t* framePatch = reinterpret_cast<uint8_t *>(inputFrame.patchData.data);

    debug = Scalar(0);
    for (int i = 0; i < inputFrame.SUB_H; i++) {
        for (int j = 0; j < inputFrame.SUB_W; j++) {
            uint8_t patch = framePatch[i*80 + j];
            if (!visited[i][j] && patch == 255) {
                int num = 0;
                shared_ptr<PlanarRegion> planarRegion = make_shared<PlanarRegion>(components);
                dfs(i, j, components, num, debug, framePatch, planarRegion, inputFrame);
                if (num > params.REGION_MIN_PATCHES && num - planarRegion->getNumOfBoundaryVertices() > params.REGION_BOUNDARY_DIFF){
                    planarRegionList.emplace_back(planarRegion);
                    components++;
                    sizes.push_back(num);
                    ids.push_back(components);
                }
            }
        }
    }
    printf("DFS Generated %d Regions\n", components);
}


void MapFrameProcessor::dfs(int x, int y, int component, int& num, Mat& debug, uint8_t* framePatch, shared_ptr<PlanarRegion> planarRegion, MapFrame inputFrame) {
    if (visited[x][y]) return;

    num++;
    visited[x][y] = true;
    region[x][y] = component;
    Vec6f patch = inputFrame.getRegionOutput().at<Vec6f>(x,y);
    planarRegion->addPatch(Vector3f(patch[0], patch[1], patch[2]), Vector3f(patch[3], patch[4], patch[5]));
    circle(debug, Point((y)*10, (x)*10), 2, Scalar((component+1)*231 % 255,(component+1)*123 % 255,(component+1)*312 % 255), -1);

    int count = 0;
    for (int i = 0; i < 8; i++) {
        if (x + adx[i] < inputFrame.SUB_H - 1 && x + adx[i] > 1 && y + ady[i] < inputFrame.SUB_W - 1 && y + ady[i] > 1) {
            uint8_t newPatch = framePatch[ (x + adx[i])*80 + (y + ady[i])];
            // printf("(%d, %d, %d) ", x+adx[i], y + ady[i], newPatch);
            if (newPatch == 255){
                count++;
                dfs(x+adx[i], y+ady[i], component, num, debug, framePatch, planarRegion, inputFrame);
            }
        }
    }
    // printf("\n");
    if (count != 8) {
        planarRegion->insertBoundaryVertex(Vector3f(patch[3], patch[4], patch[5]));
        circle(debug, Point((y)*10, (x)*10), 2, Scalar(255,255,255), -1);
    }

    // imshow("Debugger", debug);
    // waitKey(8);
}

// PlanarRegionPolygonizer
// PlanarRegionSegmentationMessage
//