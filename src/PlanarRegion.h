#ifndef SRC_PLANARREGION_H
#define SRC_PLANARREGION_H

#include <Eigen/Dense>
#include <vector>
#include <iostream>


using namespace Eigen;
using namespace std;

class PlanarRegion {
private:
    Vector3f normal;
    Vector3f center;
    vector<Vector3f> patchCentroids;
    vector<Vector3f> boundaryVertices;
    int numPatches;
    int id;

public:
    PlanarRegion(int id);
    void addPatch(Vector3f normal, Vector3f center);
    void insertBoundaryVertex(Vector3f vertex);
    void getClockWise2D(vector<Vector2f>& points);

    int getNumOfBoundaryVertices();
    Vector3f getPCANormal();
    Vector3f getMeanNormal();
    Vector3f getMeanCenter();
    vector<Vector3f> getVertices();

    int getNumPatches();

    int getId();
};


#endif //SRC_PLANARREGION_H
