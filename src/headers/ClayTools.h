//
// Created by quantum on 3/11/22.
//

#ifndef MAP_SENSE_CLAYTOOLS_H
#define MAP_SENSE_CLAYTOOLS_H

#include "glm/glm.hpp"
#include "Eigen/Core"

namespace ClayTools
{
   glm::mat4 EigenToClay(const Eigen::Matrix4f& transform, float scalar = 1.0f);
}



#endif //MAP_SENSE_CLAYTOOLS_H
