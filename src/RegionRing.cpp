//
// Created by quantum on 1/14/21.
//

#include "RegionRing.h"

RegionRing::RegionRing(int id) {
    this->id = id;
}

void RegionRing::insertBoundaryVertex(Vector3f pos){
    this->boundaryIndices.push_back(pos);
}

int RegionRing::getNumOfVertices() const {
    return boundaryIndices.size();
}

int RegionRing::getId() const {
    return id;
}
