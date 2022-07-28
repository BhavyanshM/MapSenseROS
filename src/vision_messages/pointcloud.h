#pragma once

#include "Eigen/Core"

namespace mapsense
{
   struct PointCloud
   {
      std::vector<Eigen::Vector3f> cloud;
   };
}
