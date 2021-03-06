#include "PlanarRegion.h"


PlanarRegion::PlanarRegion(int id){
    numPatches = 0;
    this->id = id;
}

Vector3f PlanarRegion::getMeanNormal() {
    return this->normal / (float) this->numPatches;
}

Vector3f PlanarRegion::getPCANormal(){
    Matrix<float, 3, Dynamic> patchMatrix(3, patchCentroids.size());
    for(int i = 0; i<patchCentroids.size(); i++) {
        patchMatrix.col(i) = patchCentroids[i];
    }
    Vector3f centroid(patchMatrix.row(0).mean(), patchMatrix.row(1).mean(), patchMatrix.row(2).mean());
    patchMatrix.row(0).array() -= centroid(0); patchMatrix.row(1).array() -= centroid(1); patchMatrix.row(2).array() -= centroid(2);
    auto svd = patchMatrix.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    Vector3f plane_normal = svd.matrixU().rightCols<1>();
    return plane_normal;
}

Vector3f PlanarRegion::getMeanCenter() {
    return this->center / (float) this->numPatches;
}

vector<Vector3f> PlanarRegion::getVertices() {
    return boundaryVertices;
}

vector<Vector2i> PlanarRegion::getLeafPatches() {
    return leafPatches;
}

int PlanarRegion::getNumOfBoundaryVertices() {
    return this->boundaryVertices.size();
}

void PlanarRegion::addPatch(Vector3f normal, Vector3f center) {
    this->normal += normal;
    this->center += center;
    this->patchCentroids.emplace_back(center);
    this->numPatches += 1;
    // printf("Region[%d]:(%d), Center:(%.3lf, %.3lf, %.3lf), Normal:(%.3lf, %.3lf, %.3lf)\n", getId(), getNumPatches(), this->center[0], this->center[1], this->center[2], this->normal[0], this->normal[1], this->normal[2]);
}

void PlanarRegion::insertBoundaryVertex(Vector3f vertex) {
    this->boundaryVertices.push_back(vertex);
}

void PlanarRegion::insertLeafPatch(Vector2i pos){
    this->leafPatches.push_back(pos);
}

int PlanarRegion::getNumPatches() {
    return numPatches;
}

int PlanarRegion::getId() {
    return id + 1;
}

void PlanarRegion::getClockWise2D(vector<Vector2f>& points){
    printf("Getting ClockWise 2D\n");
    Vector3f center = this->getMeanCenter();
    Vector3f normal = this->getMeanNormal();

    for(int i = 0; i< this->getNumOfBoundaryVertices(); i++){
        Vector3f vec = this->getVertices()[i] - center;
        Vector3f up(0,0,1);
        Vector3f dir(0.01f*normal.x(), 0.01f*normal.y(), 0.01f*normal.z());
        Vector3f axis = dir.cross(up).normalized();
        float angle = acos(up.dot(dir)/(up.norm()*dir.norm()));

        if (!isnan(axis.x()) && !isnan(axis.y()) && !isnan(axis.z())){

            AngleAxisf angleAxis(angle, axis);
            Quaternionf quat(angleAxis);
            Vector3f meshVec = quat._transformVector(vec);
            if(meshVec.norm() < 4){
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
    printf("ClockWise 2D Generated\n");
}

