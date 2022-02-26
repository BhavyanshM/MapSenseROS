//
// Created by quantum on 2/25/22.
//

#ifndef MAP_SENSE_IMGUITOOLS_H
#define MAP_SENSE_IMGUITOOLS_H

#include "MapsenseHeaders.h"
#include "Eigen/Core"
#include "imgui.h"
#include "implot.h"

class ImGuiTools
{
   public:
      static void ScatterPlotXY(const std::vector<Eigen::Vector2f>& points);
};

#endif //MAP_SENSE_IMGUITOOLS_H
