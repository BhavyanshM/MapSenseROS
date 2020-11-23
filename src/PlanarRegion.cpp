#include "PlanarRegion.h"


PlanarRegion::PlanarRegion(int id){
    numPatches = 0;
    this->id = id;
}

Vector3f PlanarRegion::getNormal() {
    return this->normal / (float) this->numPatches;
}

Vector3f PlanarRegion::getCenter() {
    return this->center / (float) this->numPatches;
}

vector<Vector2f> PlanarRegion::getVertices() {
    return vertices;
}

int PlanarRegion::getNumOfVertices() {
    return this->vertices.size();
}

void PlanarRegion::addPatch(Vector3f normal, Vector3f center) {
    this->normal += normal;
    this->center += center;
    this->numPatches += 1;
    // printf("Region[%d]:(%d), Center:(%.3lf, %.3lf, %.3lf), Normal:(%.3lf, %.3lf, %.3lf)\n", getId(), getNumPatches(), this->center[0], this->center[1], this->center[2], this->normal[0], this->normal[1], this->normal[2]);
}

void PlanarRegion::insertVertex(Vector2f vertex) {
    this->vertices.push_back(vertex);
}

int PlanarRegion::getNumPatches() {
    return numPatches;
}

int PlanarRegion::getId() {
    return id;
}

