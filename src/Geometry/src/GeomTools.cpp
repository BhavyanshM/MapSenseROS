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

void GeomTools::compressPointSetLinear(shared_ptr<PlanarRegion> region)
{
//   printf("Extended Boundary Size: %d\n", region->getNumOfBoundaryVertices());
   vector<Vector3f> boundary = region->getBoundaryVertices();
   region->boundaryVertices.clear();
   for(uint16_t i = 0; i<boundary.size() - 12; i++){
      if( ((boundary[i] - boundary[i+12/2]).normalized().dot((boundary[i+12/2] - boundary[i+12]).normalized())) < 0.5f)
      {
         region->boundaryVertices.emplace_back(boundary[i+1]);
      }
   }
//   printf("Reduced Boundary Size: %d\n", region->getNumOfBoundaryVertices());
}