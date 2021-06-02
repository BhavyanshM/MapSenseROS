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

      RigidBodyTransform(const Eigen::Matrix4d& matrix);

      RigidBodyTransform(const RigidBodyTransform& other) : matrix(other.matrix){}

      RigidBodyTransform(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation);

      RigidBodyTransform(const Eigen::Vector3d& eulerAngles, const Eigen::Vector3d& translation);

      void setToInverse();

      RigidBodyTransform getInverse();

      Eigen::Matrix4d getMatrix();

      void setMatrix(const Eigen::Matrix4d& matrix);

      void appendLeft(const RigidBodyTransform& transform);

      void appendRight(const RigidBodyTransform& transform);

      Eigen::Vector3d transformVector(const Eigen::Vector3d& vector);

      void print();

      void setRotationAndTranslation(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation);

      void setRotationAndTranslation(const Eigen::Vector3d& eulerAngles, const Eigen::Vector3d& translation);

      void setRotationAndTranslation(const Eigen::Quaterniond& orientation, const Eigen::Vector3d& translation);

      Eigen::Vector3d getTranslation();

      Eigen::Quaterniond getQuaternion();
};

#endif //RIGIDBODYTRANSFORM_H
