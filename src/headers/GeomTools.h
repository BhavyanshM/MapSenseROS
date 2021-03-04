//
// Created by quantum on 3/3/21.
//

#ifndef GEOMTOOLS_H
#define GEOMTOOLS_H

#include "Eigen/Dense"

using namespace Eigen;

class GeomTools
{
   public:
      static Matrix3f getRotationFromAngleApproximations(Vector3f eulerAngles);
};

#endif //GEOMTOOLS_H
