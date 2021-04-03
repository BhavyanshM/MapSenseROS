//
// Created by quantum on 4/2/21.
//

#ifndef RIGIDBODYTRANSFORM_H
#define RIGIDBODYTRANSFORM_H

#include "Eigen/Dense"

class RigidBodyTransform
{
   private:
      Eigen::Matrix4d matrix;
   public:
      RigidBodyTransform();

      RigidBodyTransform(Eigen::Matrix4d matrix);

      RigidBodyTransform(Eigen::Vector3d eulerAngles, Eigen::Vector3d translation);

      void getInverseTransform(RigidBodyTransform& transformToPack);

      void setToInverse();

      const Eigen::Matrix4d& getMatrix() const;

      void setMatrix(const Eigen::Matrix4d& matrix);

      void appendLeft(RigidBodyTransform& transform);

      void appendRight(RigidBodyTransform& transform);

   public:
};

#endif //RIGIDBODYTRANSFORM_H
