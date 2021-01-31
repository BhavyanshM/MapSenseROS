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

     int NUM_SKIP_EDGES = 6;

     bool SHOW_BOUNDARIES = true;
     bool SHOW_PATCHES = true;

};


#endif //SRC_APPLICATIONSTATE_H
