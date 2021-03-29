//
// Created by quantum on 3/28/21.
//

#ifndef NODE_H
#define NODE_H

#include <Eigen/Dense>

class Node
{
   public:
      Eigen::Vector3f point;
      Node* parent = nullptr;
      Node* right = nullptr;
      Node* left = nullptr;

      explicit Node(Eigen::Vector3f point);
};

#endif //NODE_H
