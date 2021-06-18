//
// Created by quantum on 1/14/21.
//

#ifndef REGIONHOLE_H
#define REGIONHOLE_H

#include <Eigen/Dense>
#include "MapsenseHeaders.h"

using namespace Eigen;
using namespace std;

class RegionRing
{
   public:
      vector<Vector3f> boundaryVertices;
      vector<Vector2i> boundaryIndices;
      int id;

      RegionRing(int id);

      void insertBoundaryVertex(Vector3f pos);

      void insertBoundaryIndex(Vector2i pos);

      int getNumOfVertices() const;

      int getId() const;
};

#endif //REGIONHOLE_H
