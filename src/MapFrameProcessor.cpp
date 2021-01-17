#include "MapFrameProcessor.h"


void MapFrameProcessor::printPatchGraph(MapFrame inputFrame){
    for(int i = 0; i<inputFrame.SUB_H; i++){
        for(int j = 0; j<inputFrame.SUB_W; j++){
            uint8_t current = inputFrame.patchData.at<uint8_t>(i,j);
            if (boundary[i][j] == 1){
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
    this->params = params;

    /* For initial development only. Delete all old previous regions before inserting new ones. Old and new regions should be fused instead. */
    for(int i = 0; i<planarRegionList.size(); i++){
        planarRegionList[i].reset();
    }
    planarRegionList.clear();

    int components = 0;
    memset(visited, 0, sizeof(bool) * 60 * 80);
    memset(region, 0, sizeof(int) * 60 * 80);
    memset(boundary, 0, sizeof(bool) * 60 * 80);

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
                }
            }
        }
    }
    printf("DFS Generated %d Regions\n", components);

//    printPatchGraph(inputFrame);

    memset(visited, 0, sizeof(bool) * 60 * 80);
    findBoundaryAndHoles(inputFrame, planarRegionList);

}

void MapFrameProcessor::findBoundaryAndHoles(MapFrame frame, vector<shared_ptr<PlanarRegion>> &planarRegionList) {
    for(int i = 0; i<planarRegionList.size(); i++){
        int components = 0;
        vector<Vector2i> leafPatches = planarRegionList[i]->getLeafPatches();
        for(int j = 0; j<leafPatches.size(); j++){
            int num = 0;
            shared_ptr<RegionHole> regionHole = make_shared<RegionHole>(components);
            boundary_dfs(leafPatches[j].x(), leafPatches[j].y(), components, num, debug, regionHole, frame);
            if(num > 3){
                planarRegionList[i]->rings.emplace_back(regionHole);
                components++;
//                printf("Region ID: %d, RegionHole ID:%d\n", planarRegionList[i]->getId(), regionHole->getId());
//                printf("Num Points: %d\n", num);
            }
        }

        auto comp = [](const shared_ptr<RegionHole> a, const shared_ptr<RegionHole> b){ return a->getNumOfVertices() > b->getNumOfVertices();};
        sort(planarRegionList[i]->rings.begin(), planarRegionList[i]->rings.end(), comp);
        printf("Region:%d, NumRegions:%d\n", planarRegionList[i]->getId(), planarRegionList[i]->rings.size());
        for(int j = 0; j<planarRegionList[i]->rings.size(); j++){
            printf("RegionSize:%d\n", planarRegionList[i]->rings[j]->getNumOfVertices());
        }
    }
}

void MapFrameProcessor::dfs(int x, int y, int component, int& num, Mat& debug, uint8_t* framePatch, shared_ptr<PlanarRegion> planarRegion, MapFrame inputFrame) {
    if (visited[x][y]) return;

    num++;
    visited[x][y] = true;
    region[x][y] = component;
    Vec6f patch = inputFrame.getRegionOutput().at<Vec6f>(x,y);
    planarRegion->addPatch(Vector3f(patch[0], patch[1], patch[2]), Vector3f(patch[3], patch[4], patch[5]));
    if(params.SHOW_PATCHES)circle(debug, Point((y)*10, (x)*10), 2, Scalar((component+1)*231 % 255,(component+1)*123 % 255,(component+1)*312 % 255), -1);

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
        boundary[x][y] = true;
        planarRegion->insertLeafPatch(Vector2i(x,y));
        if(params.SHOW_BOUNDARIES) circle(debug, Point((y)*10, (x)*10), 2, Scalar(255,255,255), -1);
    }

    // imshow("Debugger", debug);
    // waitKey(8);
}

void MapFrameProcessor::boundary_dfs(int x, int y, int component, int& num, Mat& debug, shared_ptr<RegionHole> regionHole, MapFrame inputFrame){
    if (visited[x][y]) return;

    num++;
    visited[x][y] = true;
    Vec6f patch = inputFrame.getRegionOutput().at<Vec6f>(x,y);
    regionHole->insertBoundaryVertex(Vector3f(patch[0], patch[1], patch[2]));
    if(params.SHOW_BOUNDARIES) circle(debug,
                                      Point((y)*10, (x)*10), 2,
                                      Scalar((component+1)*130 % 255,(component+1)*227 % 255,(component+1)*332 % 255),
                                      -1);
//    printf("%d, %d\n", x, y);

    for (int i = 0; i < 8; i++) {
        if (x + adx[i] < inputFrame.SUB_H - 1 && x + adx[i] > 1 && y + ady[i] < inputFrame.SUB_W - 1 && y + ady[i] > 1) {
            if (boundary[x+adx[i]][y+ady[i]] == 1){
                boundary_dfs(x+adx[i], y+ady[i], component, num, debug, regionHole, inputFrame);
            }
        }
    }
}
