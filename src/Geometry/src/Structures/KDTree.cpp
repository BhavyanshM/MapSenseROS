//
// Created by quantum on 3/28/21.
//

#include "../../include/KDTree.h"

KDTree::KDTree(){}

KDTree::KDTree(Eigen::Vector3f rootPoint)
{
   this->root = new KDNode(rootPoint);
}

KDNode *KDTree::insert(KDNode *node, Eigen::Vector3f point, uint8_t level)
{
   uint8_t indexToCheck = level % this->dim;

   if (node == nullptr)
   {
      node = new KDNode(point);
   } else if (point(indexToCheck) <= node->point(indexToCheck))
   {
      node->left = insert(node->left, point, level + 1);
   } else if (point(indexToCheck) > node->point(indexToCheck))
   {
      node->right = insert(node->right, point, level + 1);
   }
   return node;
}

KDNode *KDTree::nearestNeighbor(KDNode *node, Eigen::Vector3f point, uint8_t level)
{

   if (node == nullptr)
   {
      printf("Reached NULL\n");
      return nullptr;
   }
   printf("Called:(%.2f, %.2f, %.2f)\n", node->point.x(), node->point.y(), node->point.z());

   uint8_t indexToCheck = level % this->dim;

   if (point(indexToCheck) <= node->point(indexToCheck))
   {
      printf("Goind Left\n");
      nextNode = node->left;
      otherNode = node->right;
   } else
   {
      printf("Goind Right\n");
      nextNode = node->right;
      otherNode = node->left;
   }

   KDNode *temp = nearestNeighbor(nextNode, point, level + 1);
   KDNode *best = closest(point, temp, node);

   float radialDist = (point - best->point).squaredNorm();
   float perpDist = point(indexToCheck) - node->point(indexToCheck);

   if (radialDist >= perpDist * perpDist)
   {
      printf("Checking Perp\n");
      temp = nearestNeighbor(otherNode, point, level + 1);
      best = closest(point, temp, best);
   }

   return best;
}

KDNode *KDTree::closest(const Eigen::Vector3f& point, KDNode *first, KDNode *second)
{
   if(first == nullptr) return second;
   if(second == nullptr) return first;

   float sqDistToFirst = (point - first->point).squaredNorm();
   float sqDistToSecond = (point - second->point).squaredNorm();

   printf("Distance of (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f) is: %.2f ----- ", point.x(), point.y(), point.z(), first->point.x(), first->point.y(), first->point.z(), sqDistToFirst);
   printf("Distance of (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f) is: %.2f\n", point.x(), point.y(), point.z(), second->point.x(), second->point.y(), second->point.z(), sqDistToSecond);

   return (sqDistToFirst <= sqDistToSecond) ? first : second;
}

