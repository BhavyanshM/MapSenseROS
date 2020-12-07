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

vector<Vector3f> PlanarRegion::getVertices() {
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

void PlanarRegion::insertVertex(Vector3f vertex) {
    this->vertices.push_back(vertex);
}

int PlanarRegion::getNumPatches() {
    return numPatches;
}

int PlanarRegion::getId() {
    return id;
}

void PlanarRegion::getClockWise2D(vector<Vector2f>& points){
    // printf("START \t");
    Vector3f center = this->getCenter();
    Vector3f normal = this->getNormal();

    for(int i = 0; i<this->getNumOfVertices(); i++){
        Vector3f vec = this->getVertices()[i] - center;
        Vector3f up(0,0,1);
        Vector3f dir(0.01f*normal.x(), 0.01f*normal.y(), 0.01f*normal.z());
        Vector3f axis = dir.cross(up).normalized();
        float angle = acos(up.dot(dir)/(up.norm()*dir.norm()));

        if (!isnan(axis.x()) && !isnan(axis.y()) && !isnan(axis.z())){

            AngleAxisf angleAxis(angle, axis);
            Quaternionf quat(angleAxis);
            Vector3f meshVec = quat._transformVector(vec);
            if(meshVec.norm() < 2){
                points.emplace_back(Vector2f(2*meshVec.x(), 2*meshVec.y()));
            }else{
                printf("Singularity:(%.2lf, %.2lf, %.2lf)\n", meshVec.x(), meshVec.y(), meshVec.norm());
            }
        }
    }
    // printf("Points:%d\n", points.size());
    Vector2f north(1, 0);
    north.normalize();
    sort(points.begin(), points.end(), [=](const Vector2f& a, Vector2f& b) -> bool
    {
        return atan2(a.y(), a.x()) < atan2(b.y(), b.x());
    });
    points.emplace_back(Vector2f(points[0].x(), points[0].y()));
    // for(int i = 0; i<points.size(); i++){
    //     printf("%.2lf, %.2lf\n", points[i].x(), points[i].y());
    // }
    // printf("STOP\n");
}

