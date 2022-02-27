//
// Created by quantum on 2/25/22.
//

#ifndef MAP_SENSE_IMGUITOOLS_H
#define MAP_SENSE_IMGUITOOLS_H

#include "Eigen/Core"
#include "imgui.h"
#include "implot.h"
#include "vector"

class ImGuiTools
{
   public:
      static void ScatterPlotXY(const std::vector<Eigen::Vector2f>& points);

      static Eigen::Vector2f ScatterPlotRegionSegments(const std::vector<Eigen::Vector2f>& points, const std::vector<int>& segmentIDs);
};

#endif //MAP_SENSE_IMGUITOOLS_H
