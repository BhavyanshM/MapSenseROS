//
// Created by quantum on 4/5/21.
//

#ifndef LIEALGEBRA_H
#define LIEALGEBRA_H

#include "Eigen/Dense"

class LieAlgebra
{
   private:
      Eigen::Matrix3d matrix;
      Eigen::Vector3d crossVector;

   public:
      LieAlgebra();

      LieAlgebra(Eigen::Matrix3d matrix);

      LieAlgebra(Eigen::Vector3d eulerAngles);

      void getInverseTransform(LieAlgebra& transformToPack);

      void setToInverse();

      LieAlgebra getInverse();

      const Eigen::Matrix3d& getMatrix() const;

      void setMatrix(const Eigen::Matrix3d& matrix);

      void appendLeft(LieAlgebra& transform);

      void appendRight(LieAlgebra& transform);

      static Eigen::Matrix3d exp(LieAlgebra& rotation);

      const Eigen::Vector3d& vee() const;
};

#endif //LIEALGEBRA_H
