//
// Created by quantum on 1/14/21.
//

#ifndef REGIONHOLE_H
#define REGIONHOLE_H

#include <Eigen/Dense>
#include "vector"

class RegionRing
{
   public:
      std::vector<Eigen::Vector3f> boundaryVertices;
      std::vector<Eigen::Vector2i> boundaryIndices;
      int id;

      RegionRing(int id);

      void insertBoundaryVertex(Eigen::Vector3f pos);

      void insertBoundaryIndex(Eigen::Vector2i pos);

      int getNumOfVertices() const;

      int getId() const;
};

#endif //REGIONHOLE_H
