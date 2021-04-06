#include "Rotation.h"

Rotation::Rotation()
{
   this->matrix.setIdentity();
}

Rotation::Rotation(Eigen::Matrix3d matrix)
{
   this->matrix = matrix;
}

Rotation::Rotation(double eX, double eY, double eZ)
{
   this->matrix =
         Eigen::AngleAxisd(eX, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(eY, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(eZ, Eigen::Vector3d::UnitZ());
}

void Rotation::setToInverse()
{
   this->matrix = this->matrix.transpose();
}

Rotation Rotation::getInverse()
{
   Rotation rotation;
   rotation.matrix = matrix;
   return matrix;
}

const Eigen::Matrix3d& Rotation::getMatrix() const
{
   return matrix;
}

void Rotation::setMatrix(const Eigen::Matrix3d& matrix)
{
}

void Rotation::appendLeft(Rotation& rotation)
{
   this->matrix = rotation.matrix * rotation.matrix;
}

void Rotation::appendRight(Rotation& rotation)
{
   this->matrix = this->matrix * rotation.matrix;
}

LieAlgebra log(Rotation& rotation)
{
   double psi = acos((rotation.getMatrix().trace() - 1) / 2);
   Eigen::Matrix3d element = psi * (rotation.getMatrix() - rotation.getMatrix().transpose()) / (2*sin(psi));
   return LieAlgebra(element);
}

Rotation exp(LieAlgebra& element)
{
   Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
   rotation += sin(element.vee().norm())/element.vee().norm() * element.getMatrix()
         + (1 - cos(element.vee().norm()))/element.vee().squaredNorm() * element.getMatrix() * element.getMatrix();
   return rotation;
}