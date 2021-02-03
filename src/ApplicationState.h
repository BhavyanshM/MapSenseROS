//
// Created by quantum on 12/24/20.
//

#ifndef SRC_APPLICATIONSTATE_H
#define SRC_APPLICATIONSTATE_H

#include <string>

#define SHOW_INPUT_DEPTH 1
#define SHOW_FILTERED_DEPTH 2
#define SHOW_REGION_COMPONENTS 0
#define SHOW_INPUT_COLOR 3


using namespace std;

class ApplicationState {
public:
    const string &getDepthFile() const;

    void setDepthFile(const string &depthFile);

    const string &getColorFile() const;

    void setColorFile(const string &colorFile);

    void update();

private:
    string depthFile = "/data/Depth_L515.png";
    string colorFile = "/data/Color_L515.png";

public:

    float MERGE_ANGULAR_THRESHOLD = 0.5;

    float MERGE_DISTANCE_THRESHOLD = 0.09;
    float FILTER_DISPARITY_THRESHOLD = 2000;
    float MAGNUM_PATCH_SCALE = 0.01;

    int REGION_MIN_PATCHES = 20;
    int REGION_BOUNDARY_DIFF = 20;

    /*
     * NOTE: The following parameters should meet these requirements.
     * a) InputHeight should be divisible by (KernelResLevel * AspectRationHeight)
     * b) InputWidth should be divisible by (KernelResLevel * AspectRationWidth)
     * */
    int levels[7] = {8,10,16,20,32,40,80};
    int KERNEL_SLIDER_LEVEL = 5;
    int KERNEL_RESOLUTION_LEVEL = levels[KERNEL_SLIDER_LEVEL - 1];
    int ASPECT_RATIO_HEIGHT = 3;
    int ASPECT_RATIO_WIDTH = 4;
    int INPUT_HEIGHT = 480;
    int INPUT_WIDTH = 640;
    int SUB_H = ASPECT_RATIO_HEIGHT * KERNEL_RESOLUTION_LEVEL;
    int SUB_W = ASPECT_RATIO_WIDTH * KERNEL_RESOLUTION_LEVEL;
    int PATCH_HEIGHT = (int) INPUT_HEIGHT / (SUB_H);
    int PATCH_WIDTH = (int) INPUT_WIDTH / (SUB_W);


    int NUM_SKIP_EDGES = 6;

    bool SHOW_BOUNDARIES = true;
    bool SHOW_PATCHES = true;

};


#endif //SRC_APPLICATIONSTATE_H
