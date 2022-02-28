//
// Created by quantum on 4/2/21.
//

#include "RigidBodyTransform.h"

RigidBodyTransform::RigidBodyTransform()
{
   matrix = Eigen::Matrix4d::Identity();
}

RigidBodyTransform::RigidBodyTransform(const Eigen::Matrix4d& mat)
{
   matrix = mat;
}

void RigidBodyTransform::setToInverse()
{
   matrix.block<3, 1>(0, 3) = -matrix.block<3, 3>(0, 0).transpose() * matrix.block<3, 1>(0, 3);
   matrix.block<3, 3>(0, 0) = matrix.block<3, 3>(0, 0).transpose();
}

RigidBodyTransform RigidBodyTransform::getInverse()
{
   RigidBodyTransform transformToPack;
   transformToPack.matrix.block<3, 3>(0, 0) = matrix.block<3, 3>(0, 0).transpose();
   transformToPack.matrix.block<3, 1>(0, 3) = -matrix.block<3, 3>(0, 0).transpose() * matrix.block<3, 1>(0, 3);
   return transformToPack;
}

RigidBodyTransform::RigidBodyTransform(const Eigen::Vector3d& eulerAngles, const Eigen::Vector3d& translation)
{

   Eigen::Matrix3d rotation;
   rotation = Eigen::AngleAxisd((double) eulerAngles.x(), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd((double) eulerAngles.y(), Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd((double) eulerAngles.z(), Eigen::Vector3d::UnitZ());
   matrix.setIdentity();
   matrix.block<3, 3>(0, 0) = rotation;
   matrix.block<3, 1>(0, 3) = translation;
}

void RigidBodyTransform::setRotationAndTranslation(const Eigen::Vector3d& eulerAngles, const Eigen::Vector3d& translation)
{
   Eigen::Matrix3d rotation;
   rotation = Eigen::AngleAxisd((double) eulerAngles.x(), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd((double) eulerAngles.y(), Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd((double) eulerAngles.z(), Eigen::Vector3d::UnitZ());
   matrix.setIdentity();
   matrix.block<3, 3>(0, 0) = rotation;
   matrix.block<3, 1>(0, 3) = translation;
}

void RigidBodyTransform::setRotationAndTranslation(const Eigen::Quaterniond& orientation, const Eigen::Vector3d& translation)
{
   matrix.block<3, 3>(0, 0) = orientation.toRotationMatrix();
   matrix.block<3, 1>(0, 3) = translation;
}

void RigidBodyTransform::setRotationAndTranslation(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation)
{
   matrix.setIdentity();
   matrix.block<3, 3>(0, 0) = rotation;
   matrix.block<3, 1>(0, 3) = translation;
}

Eigen::Matrix4d RigidBodyTransform::getMatrix()
{
   return matrix;
}

void RigidBodyTransform::setMatrix(const Eigen::Matrix4d& matrix)
{
   this->matrix = Eigen::Matrix4d(matrix);
}

void RigidBodyTransform::multiplyLeft(const RigidBodyTransform& transform)
{
   matrix = transform.matrix * matrix;
}

void RigidBodyTransform::multiplyRight(const RigidBodyTransform& transform)
{
   matrix = matrix * transform.matrix;
}

Eigen::Vector3d RigidBodyTransform::transformVector(const Eigen::Vector3d& vector)
{
   return matrix.block<3, 3>(0, 0) * vector + matrix.block<3, 1>(0, 3);
}

void RigidBodyTransform::print()
{
   std::cout << matrix << std::endl;
}

Eigen::Vector3d RigidBodyTransform::getTranslation()
{
   return matrix.block<3, 1>(0, 3);
}

Eigen::Quaterniond RigidBodyTransform::getQuaternion()
{
   return Eigen::Quaterniond(matrix.block<3, 3>(0, 0));
}

void RigidBodyTransform::rotateX(float rad)
{
   rotate(rad, Eigen::Vector3d::UnitX());
}

void RigidBodyTransform::rotateY(float rad)
{
   rotate(rad, Eigen::Vector3d::UnitY());
}

void RigidBodyTransform::rotateZ(float rad)
{
   rotate(rad, Eigen::Vector3d::UnitZ());
}

void RigidBodyTransform::rotate(float rad, Eigen::Vector3d axis)
{
   Eigen::Matrix4d rotation = Eigen::Matrix4d::Identity();
   rotation.block<3,3>(0,0) = Eigen::AngleAxisd(rad, axis).toRotationMatrix();
   matrix =  rotation * matrix ;
}
