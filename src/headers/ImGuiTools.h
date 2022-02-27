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

      static void ScatterPlotRegionSegments(const std::vector<Eigen::Vector2f>& points, const std::vector<int>& segmentIDs);
};

#endif //MAP_SENSE_IMGUITOOLS_H
