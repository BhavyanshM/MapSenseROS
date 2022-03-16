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

      RigidBodyTransform(const Eigen::Matrix4d& matrix);

      RigidBodyTransform(const RigidBodyTransform& other) : matrix(other.matrix){}

      RigidBodyTransform(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation);

      RigidBodyTransform(const Eigen::Vector3d& eulerAngles, const Eigen::Vector3d& translation);

      void setToInverse();

      RigidBodyTransform getInverse();

      Eigen::Matrix4d getMatrix();

      void setMatrix(const Eigen::Matrix4d& matrix);

      void multiplyLeft(const RigidBodyTransform& transform);

      void multiplyRight(const RigidBodyTransform& transform);

      Eigen::Vector3d transformVector(const Eigen::Vector3d& vector);

      void print();

      void setRotationAndTranslation(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation);

      void setRotationAndTranslation(const Eigen::Vector3d& eulerAngles, const Eigen::Vector3d& translation);

      void setRotationAndTranslation(const Eigen::Quaterniond& orientation, const Eigen::Vector3d& translation);

      Eigen::Vector3d getTranslation();

      Eigen::Quaterniond getQuaternion();

      void rotateX(float angleRad);

      void rotateY(float angleRad);

      void rotateZ(float angleRad);

      void rotate(float rad, Eigen::Vector3d axis);
};

#endif //RIGIDBODYTRANSFORM_H
