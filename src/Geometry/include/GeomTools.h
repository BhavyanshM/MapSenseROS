//
// Created by quantum on 3/3/21.
//

#ifndef GEOMTOOLS_H
#define GEOMTOOLS_H

#include "Eigen/Dense"
#include "PlanarRegion.h"

using namespace Eigen;

class GeomTools
{
   public:
      static Matrix3f getRotationFromAngleApproximations(Vector3f eulerAngles);
      static Vector3f getProjectedPoint(Vector4f plane, Vector3f point);
      static void getInverseTransform(Vector3d eulerAngles, Vector3d translation, MatrixXd& transformToPack);
      static void compressPointSetLinear(shared_ptr<PlanarRegion> region);
};

#endif //GEOMTOOLS_H
