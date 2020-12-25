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
    string depthFile = "/data/Depth_Boxes.png";
    string colorFile = "/home/quantum/Workspace/Storage/Other/Temp/Color_L515.png";


};


#endif //SRC_APPLICATIONSTATE_H
