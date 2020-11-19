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

public:
    void addPatch(Vector3f normal, Vector3f center);
    void insertVertex(Vector2f vertex);
    Vector3f getNormal() const;
    Vector3f getCenter() const;
    vector<Vector2f> getVertices() const;
    int getNumOfVertices() const;
};


#endif //SRC_PLANARREGION_H
