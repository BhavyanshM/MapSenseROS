//
// Created by quantum on 2/25/22.
//
#include "ImGuiTools.h"

void ImGuiTools::ScatterPlotXY(const std::vector<Eigen::Vector2f>& points, const std::string& label, bool lines)
{
   std::vector<float> x_data, y_data;
   for(int i = 0; i<points.size(); i++)
   {
      x_data.push_back(points[i].x());
      y_data.push_back(points[i].y());
   }
   x_data.push_back(points[0].x());
   y_data.push_back(points[0].y());

   ImPlot::PlotScatter(label.c_str(), x_data.data(), y_data.data(), points.size());
   if(lines) ImPlot::PlotLine(label.c_str(), x_data.data(), y_data.data(), points.size() + 1);
}

Eigen::Vector2f ImGuiTools::ScatterPlotRegionSegments(const std::vector<Eigen::Vector2f>& points, const std::vector<int>& segmentIDs)
{
   Eigen::Vector2f mousePlotLocation;
   mousePlotLocation.setZero();
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
      if(ImPlot::IsPlotHovered())
      {
         ImPlotPoint mouse = ImPlot::GetPlotMousePos();
         mousePlotLocation.x() = mouse.x;
         mousePlotLocation.y() = mouse.y;
      }
      ImPlot::PlotScatter("Region 2D", x_data.data(), y_data.data(), x_data.size());
      ImPlot::PlotLine("Region 2D", x_data.data(), y_data.data(), points.size() - 1);
      ImPlot::EndPlot();
   }
   ImGui::End();
   return mousePlotLocation;
}

void ImGuiTools::GetDropDownSelection(std::string label, std::string name, int& current, int total)
{
   if(ImGui::BeginCombo(label.c_str(), (name + " ", std::to_string(current)).c_str()))
   {
      for (int n = 0; n < total; n++)
      {
         bool is_selected = (current == n);
         if (ImGui::Selectable((name + " " + std::to_string(n)).c_str(), is_selected))
            current = n;
         if (is_selected)
            ImGui::SetItemDefaultFocus();
      }
      ImGui::EndCombo();
   }
}

void ImGuiTools::GetDropDownSelection(std::string label, std::vector<std::string> names, int& current)
{
   if(ImGui::BeginCombo(label.c_str(), (names[current]).c_str()))
   {
      for (int n = 0; n < names.size(); n++)
      {
         bool is_selected = (current == n);
         if (ImGui::Selectable((names[n]).c_str(), is_selected))
            current = n;
         if (is_selected)
            ImGui::SetItemDefaultFocus();
      }
      ImGui::EndCombo();
   }
}

