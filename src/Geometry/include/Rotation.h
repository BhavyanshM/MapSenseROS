//
// Created by quantum on 4/5/21.
//

#ifndef ROTATION_H
#define ROTATION_H

#include <LieAlgebra.h>
#include "Eigen/Dense"

class Rotation
{
   private:
      Eigen::Matrix3d matrix;
   public:
      Rotation();

      Rotation(Eigen::Matrix3d matrix);

      Rotation(double eX, double eY, double eZ);

      void getInverseTransform(Rotation& transformToPack);

      void setToInverse();

      Rotation getInverse();

      const Eigen::Matrix3d& getMatrix() const;

      void setMatrix(const Eigen::Matrix3d& matrix);

      void appendLeft(Rotation& transform);

      void appendRight(Rotation& transform);

      static LieAlgebra exp(Rotation& rotation);
};

#endif //ROTATION_H
