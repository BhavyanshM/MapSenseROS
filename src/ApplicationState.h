//
// Created by quantum on 12/24/20.
//

#ifndef SRC_APPLICATIONSTATE_H
#define SRC_APPLICATIONSTATE_H

#include <string>

using namespace std;

class ApplicationState {
public:
    const string &getDepthFile() const;

    void setDepthFile(const string &depthFile);

    const string &getColorFile() const;

    void setColorFile(const string &colorFile);

private:
    string depthFile = "/data/Depth_L515.png";
    string colorFile = "/home/quantum/Workspace/Storage/Other/Temp/Color_L515.png";

public:
     float MERGE_ANGULAR_THRESHOLD = 0.6;
     float MERGE_DISTANCE_THRESHOLD = 0.07;
     float FILTER_DISPARITY_THRESHOLD = 2000;

     float MAGNUM_PATCH_SCALE = 0.01;
     int REGION_MIN_PATCHES = 20;
     int REGION_BOUNDARY_DIFF = 20;

};


#endif //SRC_APPLICATIONSTATE_H
