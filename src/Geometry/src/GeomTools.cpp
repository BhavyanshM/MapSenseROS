//
// Created by quantum on 3/3/21.
//

#include "../include/GeomTools.h"

Matrix3f GeomTools::getRotationFromAngleApproximations(Vector3f eulerAngles)
{
   float alpha = eulerAngles.x();
   float beta = eulerAngles.y();
   float gamma = eulerAngles.z();
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

Vector3f GeomTools::getProjectedPoint(Vector4f plane, Vector3f point)
{
   Vector3f normal = plane.block<3,1>(0,0).normalized();
   return point - normal * (normal.dot(point) + plane(3)/ plane.block<3,1>(0,0).norm());
}

