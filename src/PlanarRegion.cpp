#include "PlanarRegion.h"

Vector3f PlanarRegion::getNormal() const {
    return this->normal / (float) this->numPatches;
}

Vector3f PlanarRegion::getCenter() const {
    return this->normal / (float) this->numPatches;
}

vector<Vector2f> PlanarRegion::getVertices() const {
    return vertices;
}

int PlanarRegion::getNumOfVertices() const {
    return this->vertices.size();
}

void PlanarRegion::addPatch(const Vector3f &normal, const Vector3f &center) {
    this->normal += normal;
    this->center += center;
    this->numPatches += 1;
}

void PlanarRegion::insertVertex(Vector2f &vertex) {
    this->vertices.push_back(vertex);
}

