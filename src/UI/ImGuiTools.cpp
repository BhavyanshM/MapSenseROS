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
   if (ImPlot::BeginPlot("Mapper Plots", ImVec2(0, 0)))
   {
      ImPlot::PlotScatter("Region 2D", x_data, y_data, points.size());
      ImPlot::PlotLine("Region 2D", x_data, y_data, points.size());
      ImPlot::EndPlot();
   }
   ImGui::End();
}