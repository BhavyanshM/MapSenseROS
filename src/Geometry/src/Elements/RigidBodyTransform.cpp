//
// Created by quantum on 4/2/21.
//

#include "RigidBodyTransform.h"

RigidBodyTransform::RigidBodyTransform()
{
   this->matrix.setIdentity();
}

RigidBodyTransform::RigidBodyTransform(Eigen::Matrix4d matrix){
   this->matrix = matrix;
}

void RigidBodyTransform::setToInverse()
{
   this->matrix.block<3, 1>(0, 3) = -this->matrix.block<3,3>(0,0).transpose() * this->matrix.block<3,1>(0,3);
   this->matrix.block<3, 3>(0, 0) = this->matrix.block<3,3>(0,0).transpose();
}

RigidBodyTransform RigidBodyTransform::getInverse()
{
   RigidBodyTransform transformToPack;
   transformToPack.matrix.block<3, 3>(0, 0) = this->matrix.block<3,3>(0,0).transpose();
   transformToPack.matrix.block<3, 1>(0, 3) = -this->matrix.block<3,3>(0,0).transpose() * this->matrix.block<3,1>(0,3);
   return transformToPack;
}

RigidBodyTransform::RigidBodyTransform(Eigen::Vector3d eulerAngles, Eigen::Vector3d translation){
   Eigen::Matrix3d rotation;
   rotation = Eigen::AngleAxisd((double) eulerAngles.x(), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd((double) eulerAngles.y(), Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd((double) eulerAngles.z(), Eigen::Vector3d::UnitZ());
   this->matrix.setIdentity();
   this->matrix.block<3,3>(0,0) = rotation;
   this->matrix.block<3,1>(0,3) = translation;
}

RigidBodyTransform::RigidBodyTransform(Eigen::Matrix3d rotation, Eigen::Vector3d translation){
   this->matrix.setIdentity();
   this->matrix.block<3,3>(0,0) = rotation;
   this->matrix.block<3,1>(0,3) = translation;
}

const Eigen::Matrix4d& RigidBodyTransform::getMatrix() const
{
   return matrix;
}

void RigidBodyTransform::setMatrix(const Eigen::Matrix4d& matrix)
{
   this->matrix = matrix;
}

void RigidBodyTransform::appendLeft(RigidBodyTransform& transform)
{
   this->matrix = this->matrix * transform.matrix;
}

void RigidBodyTransform::appendRight(RigidBodyTransform& transform)
{
   this->matrix = transform.matrix * this->matrix;
}

Eigen::Vector3d RigidBodyTransform::transformEuclidean(Eigen::Vector3d vector)
{
   return this->matrix.block<3,3>(0,0) * vector + this->matrix.block<3,1>(0,3);
}
