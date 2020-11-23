#ifndef SRC_PLANARREGION_H
#define SRC_PLANARREGION_H

#include <Eigen/Dense>
#include <vector>

using namespace Eigen;
using namespace std;

class PlanarRegion {
private:
    Vector3f normal;
    Vector3f center;
    vector<Vector2f> vertices;
    int numPatches;
    int id;

public:
    PlanarRegion(int id);
    void addPatch(Vector3f normal, Vector3f center);
    void insertVertex(Vector2f vertex);
    Vector3f getNormal();
    Vector3f getCenter();
    vector<Vector2f> getVertices();
    int getNumOfVertices();

    int getNumPatches();

    int getId();
};


#endif //SRC_PLANARREGION_H
