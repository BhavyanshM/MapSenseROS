//
// Created by quantum on 3/3/21.
//

#ifndef GEOMTOOLS_H
#define GEOMTOOLS_H

#include "Eigen/Dense"

using namespace Eigen;

class GeomTools
{
      static Matrix3f getRotationFromAngleApproximations(float alpha, float beta, float gamma);
};

#endif //GEOMTOOLS_H
