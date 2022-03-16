//
// Created by quantum on 4/2/21.
//

#include "RigidBodyTransform.h"

RigidBodyTransform::RigidBodyTransform()
{
   this->matrix = Eigen::Matrix4d::Identity();
}

RigidBodyTransform::RigidBodyTransform(const Eigen::Matrix4d& mat)
{
   this->matrix = mat;
}

void RigidBodyTransform::setToInverse()
{
   this->matrix.block<3, 1>(0, 3) = -this->matrix.block<3, 3>(0, 0).transpose() * this->matrix.block<3, 1>(0, 3);
   this->matrix.block<3, 3>(0, 0) = this->matrix.block<3, 3>(0, 0).transpose();
}

RigidBodyTransform RigidBodyTransform::getInverse()
{
   RigidBodyTransform transformToPack;
   transformToPack.matrix.block<3, 3>(0, 0) = this->matrix.block<3, 3>(0, 0).transpose();
   transformToPack.matrix.block<3, 1>(0, 3) = -this->matrix.block<3, 3>(0, 0).transpose() * this->matrix.block<3, 1>(0, 3);
   return transformToPack;
}

RigidBodyTransform::RigidBodyTransform(const Eigen::Vector3d& eulerAngles, const Eigen::Vector3d& translation)
{

   Eigen::Matrix3d rotation;
   rotation = Eigen::AngleAxisd((double) eulerAngles.x(), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd((double) eulerAngles.y(), Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd((double) eulerAngles.z(), Eigen::Vector3d::UnitZ());
   this->matrix.setIdentity();
   this->matrix.block<3, 3>(0, 0) = rotation;
   this->matrix.block<3, 1>(0, 3) = translation;
}

void RigidBodyTransform::setRotationAndTranslation(const Eigen::Vector3d& eulerAngles, const Eigen::Vector3d& translation)
{
   Eigen::Matrix3d rotation;
   rotation = Eigen::AngleAxisd((double) eulerAngles.x(), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd((double) eulerAngles.y(), Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd((double) eulerAngles.z(), Eigen::Vector3d::UnitZ());
   this->matrix.setIdentity();
   this->matrix.block<3, 3>(0, 0) = rotation;
   this->matrix.block<3, 1>(0, 3) = translation;
}

void RigidBodyTransform::setRotationAndTranslation(const Eigen::Quaterniond& orientation, const Eigen::Vector3d& translation)
{
   this->matrix.block<3, 3>(0, 0) = orientation.toRotationMatrix();
   this->matrix.block<3, 1>(0, 3) = translation;
}

void RigidBodyTransform::setRotationAndTranslation(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation)
{
   this->matrix.setIdentity();
   this->matrix.block<3, 3>(0, 0) = rotation;
   this->matrix.block<3, 1>(0, 3) = translation;
}

Eigen::Matrix4d RigidBodyTransform::getMatrix()
{
   return matrix;
}

void RigidBodyTransform::setMatrix(const Eigen::Matrix4d& matrix)
{
   this->matrix = matrix;
}

void RigidBodyTransform::multiplyLeft(const RigidBodyTransform& transform)
{
   this->matrix = transform.matrix * this->matrix;
}

void RigidBodyTransform::multiplyRight(const RigidBodyTransform& transform)
{
   this->matrix = this->matrix * transform.matrix;
}

Eigen::Vector3d RigidBodyTransform::transformVector(const Eigen::Vector3d& vector)
{
   return this->matrix.block<3, 3>(0, 0) * vector + this->matrix.block<3, 1>(0, 3);
}

void RigidBodyTransform::print()
{
//   std::cout << this->matrix << std::endl;
}

Eigen::Vector3d RigidBodyTransform::getTranslation()
{
   return this->matrix.block<3, 1>(0, 3);
}

Eigen::Quaterniond RigidBodyTransform::getQuaternion()
{
   return Eigen::Quaterniond(this->matrix.block<3, 3>(0, 0));
}

void RigidBodyTransform::rotateX(float rad)
{
   this->rotate(rad, Eigen::Vector3d::UnitX());
}

void RigidBodyTransform::rotateY(float rad)
{
   this->rotate(rad, Eigen::Vector3d::UnitY());
}

void RigidBodyTransform::rotateZ(float rad)
{
   this->rotate(rad, Eigen::Vector3d::UnitZ());
}

void RigidBodyTransform::rotate(float rad, Eigen::Vector3d axis)
{
   Eigen::Matrix4d rotation = Eigen::Matrix4d::Identity();
   rotation.block<3,3>(0,0) = Eigen::AngleAxisd(rad, axis).toRotationMatrix();
   this->matrix =  rotation * this->matrix ;
}
