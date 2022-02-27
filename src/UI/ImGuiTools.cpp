//
// Created by quantum on 2/25/22.
//
#include "ImGuiTools.h"


void ImGuiTools::ScatterPlotXY(const std::vector<Eigen::Vector2f>& points)
{
   float x_data[points.size()];
   float y_data[points.size()];
   for(int i = 0; i<points.size(); i++)
   {
      x_data[i] = points[i].x();
      y_data[i] = points[i].y();
   }
   ImGui::Begin("Plotter2D");
   if (ImPlot::BeginPlot("Mapper Plots", ImVec2(1000, 1000)))
   {
      ImPlot::PlotScatter("Region 2D", x_data, y_data, points.size());
      ImPlot::PlotLine("Region 2D", x_data, y_data, points.size());
      ImPlot::EndPlot();
   }
   ImGui::End();
}

void ImGuiTools::ScatterPlotRegionSegments(const std::vector<Eigen::Vector2f>& points, const std::vector<int>& segmentIDs)
{
   std::vector<float> x_data, y_data;
   for(int i = 0; i<points.size(); i++)
   {
      if(segmentIDs[i] == 0)
      {
         x_data.push_back(points[i].x());
         y_data.push_back(points[i].y());
      }
   }
   x_data.push_back(points[0].x());
   y_data.push_back(points[0].y());
   ImGui::Begin("Plotter2D");
   if (ImPlot::BeginPlot("Mapper Plots", ImVec2(1000, 1000)))
   {
      ImPlot::SetupAxesLimits(-2, 2, -2, 2, ImGuiCond_Once);
      ImPlot::PlotScatter("Region 2D", x_data.data(), y_data.data(), x_data.size());
      ImPlot::PlotLine("Region 2D", x_data.data(), y_data.data(), points.size() - 1);
      ImPlot::EndPlot();
   }
   ImGui::End();
}