#include "MapFrameProcessor.h"

void MapFrameProcessor::init(ApplicationState app) {
    this->debug = Mat(app.INPUT_HEIGHT,app.INPUT_WIDTH, CV_8UC3);
    this->visited = MatrixXi(app.SUB_H, app.SUB_W).setZero();
    this->boundary = MatrixXi(app.SUB_H, app.SUB_W).setZero();
    this->region = MatrixXi(app.SUB_H, app.SUB_W).setZero();
}

void MapFrameProcessor::printPatchGraph(MapFrame inputFrame){
    printf("DEBUGGER:(%d,%d)\n", inputFrame.SUB_H, inputFrame.SUB_W);
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
    this->params = params;

    /* For initial development only. Delete all old previous regions before inserting new ones. Old and new regions should be fused instead. */
    planarRegionList.clear();

//    printPatchGraph(inputFrame);

    int components = 0;

    visited.setZero();
    region.setZero();
    boundary.setZero();

    uint8_t* framePatch = reinterpret_cast<uint8_t *>(inputFrame.patchData.data);
    debug = Scalar(0);
    for (int i = 0; i < inputFrame.SUB_H; i++) {
        for (int j = 0; j < inputFrame.SUB_W; j++) {
            uint8_t patch = framePatch[i*inputFrame.SUB_W + j];
            if (!visited(i,j) && patch == 255) {
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
    visited.setZero();
    findBoundaryAndHoles(inputFrame, planarRegionList);
}

void MapFrameProcessor::dfs(int x, int y, int component, int& num, Mat& debug, uint8_t* framePatch, shared_ptr<PlanarRegion> planarRegion, MapFrame inputFrame) {
    if (visited(x,y)) return;

    num++;
    visited(x,y) = 1;
    region(x,y) = component;
    Vec6f patch = inputFrame.getRegionOutput().at<Vec6f>(x,y);
    planarRegion->addPatch(Vector3f(patch[0], patch[1], patch[2]), Vector3f(patch[3], patch[4], patch[5]));
    if(params.SHOW_PATCHES)circle(debug, Point((y)*inputFrame.PATCH_HEIGHT, (x)*inputFrame.PATCH_WIDTH), 2,
                                  Scalar((component+1)*231 % 255,(component+1)*123 % 255,(component+1)*312 % 255), -1);

    int count = 0;
    for (int i = 0; i < 8; i++) {
        if (x + adx[i] < inputFrame.SUB_H - 1 && x + adx[i] > 1 && y + ady[i] < inputFrame.SUB_W - 1 && y + ady[i] > 1) {
            uint8_t newPatch = framePatch[ (x + adx[i])*inputFrame.SUB_W + (y + ady[i])];
            if (newPatch == 255){
                count++;
                dfs(x+adx[i], y+ady[i], component, num, debug, framePatch, planarRegion, inputFrame);
            }
        }
    }
    if (count != 8) {
        boundary(x,y) = 1;
        planarRegion->insertLeafPatch(Vector2i(x,y));
        if(params.SHOW_BOUNDARIES) circle(debug, Point((y)*inputFrame.PATCH_HEIGHT, (x)*inputFrame.PATCH_WIDTH), 2, Scalar(255,255,255), -1);
    }
}

void MapFrameProcessor::findBoundaryAndHoles(MapFrame frame, vector<shared_ptr<PlanarRegion>> &planarRegionList) {
    for(int i = 0; i<planarRegionList.size(); i++){
        int components = 0;
        vector<Vector2i> leafPatches = planarRegionList[i]->getLeafPatches();
        for(int j = 0; j<leafPatches.size(); j++){
            int num = 0;
            shared_ptr<RegionRing> regionRing = make_shared<RegionRing>(components);
            boundary_dfs(leafPatches[j].x(), leafPatches[j].y(), components, num, debug, regionRing, frame);
            if(num > 3){
                planarRegionList[i]->rings.emplace_back(regionRing);
                components++;
            }
        }
        auto comp = [](const shared_ptr<RegionRing> a, const shared_ptr<RegionRing> b){ return a->getNumOfVertices() > b->getNumOfVertices();};
        sort(planarRegionList[i]->rings.begin(), planarRegionList[i]->rings.end(), comp);
        vector<Vector3f> ring = planarRegionList[i]->rings[0]->boundaryIndices;
        for(int j = 0; j<ring.size(); j++){
            planarRegionList[i]->insertBoundaryVertex(ring[j]);
        }
    }
}

void MapFrameProcessor::boundary_dfs(int x, int y, int component, int& num, Mat& debug, shared_ptr<RegionRing> regionRing, MapFrame inputFrame){
    if (visited(x,y)) return;

    num++;
    visited(x,y) = 1;
    Vec6f patch = inputFrame.getRegionOutput().at<Vec6f>(x,y);
    regionRing->insertBoundaryVertex(Vector3f(patch[3], patch[4], patch[5]));
    if(params.SHOW_BOUNDARIES) circle(debug,
                                      Point((y)*inputFrame.PATCH_HEIGHT, (x)*inputFrame.PATCH_WIDTH), 2,
                                      Scalar((component+1)*130 % 255,(component+1)*227 % 255,(component+1)*332 % 255),
                                      -1);
    for (int i = 0; i < 8; i++) {
        if (x + adx[i] < inputFrame.SUB_H - 1 && x + adx[i] > 1 && y + ady[i] < inputFrame.SUB_W - 1 && y + ady[i] > 1) {
            if (boundary(x+adx[i], y+ady[i]) == 1){
                boundary_dfs(x+adx[i], y+ady[i], component, num, debug, regionRing, inputFrame);
            }
        }
    }
}
