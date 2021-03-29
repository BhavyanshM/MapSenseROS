//
// Created by quantum on 3/28/21.
//

#ifndef QUADTREE_H
#define QUADTREE_H

#include "Node.h"

class KDTree
{
   public:
      Node* root = nullptr;
      Node* nextNode = nullptr;
      Node* otherNode = nullptr;

      uint8_t dim = 3;
      KDTree(Eigen::Vector3f rootPoint);

      Node* insert(Node* node, Eigen::Vector3f point, uint8_t level);
      Node* nearestNeighbor(Node* node, Eigen::Vector3f point, uint8_t level);
      static Node* closest(const Eigen::Vector3f& point, Node* first, Node* second);
};

#endif //QUADTREE_H
