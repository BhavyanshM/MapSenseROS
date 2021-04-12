//
// Created by quantum on 3/28/21.
//

#ifndef QUADTREE_H
#define QUADTREE_H

#include "KDNode.h"

class KDTree
{
   public:
      KDNode *root = nullptr;
      KDNode *nextNode = nullptr;
      KDNode *otherNode = nullptr;

      uint8_t dim = 3;

      KDTree();

      KDTree(Eigen::Vector3f rootPoint);

      ~KDTree()
      {
         delete root;
      }

      void insert(Eigen::Vector3f point);

      KDNode *insert(KDNode *node, Eigen::Vector3f point, uint8_t level);

      Eigen::Vector3f nearestNeighbor(Eigen::Vector3f point);

      KDNode *nearestNeighbor(KDNode *node, Eigen::Vector3f point, uint8_t level);

      static KDNode *closest(const Eigen::Vector3f& point, KDNode *first, KDNode *second);
};

#endif //QUADTREE_H
