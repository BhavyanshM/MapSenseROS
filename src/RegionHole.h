//
// Created by quantum on 1/14/21.
//

#ifndef REGIONHOLE_H
#define REGIONHOLE_H

#include <Eigen/Dense>
#include <vector>
#include <iostream>


using namespace Eigen;
using namespace std;

class RegionHole {
public:
    vector<Vector3f> boundaryIndices;
    int id;

    RegionHole(int id);
    void insertBoundaryVertex(Vector3f pos);
    int getNumOfVertices() const;
    int getId() const;

};


#endif //REGIONHOLE_H
