//
// Created by quantum on 3/3/21.
//

#include "GeomTools.h"

Matrix3f GeomTools::getRotationFromAngleApproximations(float alpha, float beta, float gamma)
{
   Matrix3f rotation = Matrix3f::Identity();
   rotation(0,1) = alpha * beta - gamma;
   rotation(0,2) = alpha * gamma + beta;
   rotation(1,0) = gamma;
   rotation(1,1) += alpha * beta * gamma;
   rotation(1,2) = beta * gamma - alpha;
   rotation(2,0) = -beta;
   rotation(2,1) = alpha;
   return rotation;
}
