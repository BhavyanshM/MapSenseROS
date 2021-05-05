//
// Created by quantum on 4/2/21.
//

#ifndef RIGIDBODYTRANSFORM_H
#define RIGIDBODYTRANSFORM_H

#include "Eigen/Dense"
#include "iostream"

class RigidBodyTransform
{
   private:
      Eigen::Matrix4d matrix;
   public:
      RigidBodyTransform();

      RigidBodyTransform(Eigen::Matrix4d matrix);

      RigidBodyTransform(Eigen::Matrix3d rotation, Eigen::Vector3d translation);

      RigidBodyTransform(Eigen::Vector3d eulerAngles, Eigen::Vector3d translation);

      void setToInverse();

      RigidBodyTransform getInverse();

      Eigen::Matrix4d getMatrix();

      void setMatrix(const Eigen::Matrix4d& matrix);

      void appendLeft(RigidBodyTransform& transform);

      void appendRight(RigidBodyTransform& transform);

      Eigen::Vector3d transformVector(Eigen::Vector3d vector);

      void print();

      void setRotationAndTranslation(Eigen::Matrix3d rotation, Eigen::Vector3d translation);
};

#endif //RIGIDBODYTRANSFORM_H
