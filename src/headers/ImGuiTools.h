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
      static void ScatterPlotXY(const std::vector<Eigen::Vector2f>& points, const std::string& label, bool lines = false, bool loop = true);

      static Eigen::Vector2f ScatterPlotRegionSegments(const std::vector<Eigen::Vector2f>& points, const std::vector<int>& segmentIDs);

      static void GetDropDownSelection(std::string label, std::string name, int& current, int total);

      static void GetDropDownSelection(std::string label, std::vector<std::string> names, int& current);

      static bool BeginPlotWindow(const std::string& name)
      {
         ImGui::Begin("Plotter 2D");
         return ImPlot::BeginPlot(name.c_str(), ImVec2(1000, 1000));
      }

      static void EndPlotWindow()
      {
         ImPlot::EndPlot();
         ImGui::End();
      }
};

#endif //MAP_SENSE_IMGUITOOLS_H
