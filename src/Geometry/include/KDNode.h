//
// Created by quantum on 3/28/21.
//

#ifndef NODE_H
#define NODE_H

#include <Eigen/Dense>

class KDNode
{
   public:
      Eigen::Vector3f point;
      KDNode* parent = nullptr;
      KDNode* right = nullptr;
      KDNode* left = nullptr;

      explicit KDNode(Eigen::Vector3f point);
      ~KDNode(){
         delete right;
         delete left;
      }
};

#endif //NODE_H
