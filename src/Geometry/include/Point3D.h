//
// Created by quantum on 4/8/21.
//

#ifndef POINT3D_H
#define POINT3D_H

#include "Eigen/Dense"

class Point3D
{
   private:
      Eigen::Vector3d data;

   public:
      Eigen::Vector4d getHomogeneous();
      void setHomogeneous(Eigen::Vector4d homo);
      double distance(Point3D other);
      Eigen::Vector3d getData();
      double getX();
      double getY();
      double getZ();
};

#endif //POINT3D_H
