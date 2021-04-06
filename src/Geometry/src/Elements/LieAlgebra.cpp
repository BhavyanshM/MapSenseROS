#include "LieAlgebra.h"

LieAlgebra::LieAlgebra()
{
   this->matrix.setIdentity();
}

LieAlgebra::LieAlgebra(Eigen::Matrix3d matrix)
{
   this->matrix = matrix;
}

LieAlgebra::LieAlgebra(Eigen::Vector3d eulerAngles)
{
   this->matrix =
         Eigen::AngleAxisd((double) eulerAngles.x(), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd((double) eulerAngles.y(), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd((double) eulerAngles.z(), Eigen::Vector3d::UnitZ());
}

const Eigen::Vector3d& LieAlgebra::vee() const
{
   return crossVector;
}

void LieAlgebra::setToInverse()
{
   this->matrix = this->matrix.transpose();
}

LieAlgebra LieAlgebra::getInverse()
{
   LieAlgebra rotation;
   rotation.matrix = matrix;
   return matrix;
}

const Eigen::Matrix3d& LieAlgebra::getMatrix() const
{
   return matrix;
}

void LieAlgebra::setMatrix(const Eigen::Matrix3d& matrix)
{
}

void LieAlgebra::appendLeft(LieAlgebra& rotation)
{
   this->matrix = rotation.matrix * rotation.matrix;
}

void LieAlgebra::appendRight(LieAlgebra& rotation)
{
   this->matrix = this->matrix * rotation.matrix;
}

Eigen::Matrix3d log(LieAlgebra& rotation)
{
   double psi = acos((rotation.getMatrix().trace() - 1) / 2);
   Eigen::Matrix3d element = psi * (rotation.getMatrix() - rotation.getMatrix().transpose()) / (2*sin(psi));
   return element;
}