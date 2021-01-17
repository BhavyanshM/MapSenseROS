//
// Created by quantum on 1/14/21.
//

#include "RegionHole.h"

RegionHole::RegionHole(int id) {
    this->id = id;
}

void RegionHole::insertBoundaryVertex(Vector3f pos){
    this->boundaryIndices.push_back(pos);
}

int RegionHole::getNumOfVertices() const {
    return boundaryIndices.size();
}

int RegionHole::getId() const {
    return id;
}
