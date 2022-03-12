//
// Created by quantum on 3/11/22.
//

#include "ClayTools.h"

namespace ClayTools
{
   glm::mat4 EigenToClay(const Eigen::Matrix4f& transform, float scalar)
   {
      glm::mat4 transformToPack;
      for (int i = 0; i < 4; ++i)
         for (int j = 0; j < 4; ++j)
            transformToPack[j][i] = transform(i, j);
      transformToPack[3][0] *= scalar;
      transformToPack[3][1] *= scalar;
      transformToPack[3][2] *= scalar;
      return transformToPack;
   }

}

