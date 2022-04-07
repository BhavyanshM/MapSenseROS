#include "ImGuiLayout.h"

void ImGuiLayout::getImGuiAppLayout(ApplicationState& appState)
{
   ImGui::Checkbox("ROS Enabled", &appState.ROS_ENABLED);
   ImGui::Checkbox("Planar Regions Enabled", &appState.DEPTH_PLANAR_REGIONS_ENABLED);
   ImGui::Checkbox("ICP Enabled", &appState.ICP_ODOMETRY_ENABLED);
   ImGui::Checkbox("Stereo Odom Enabled", &appState.STEREO_ODOMETRY_ENABLED);
   ImGui::Checkbox("SLAM Enabled", &appState.SLAM_ENABLED);
   ImGui::Checkbox("Dataset Enabled", &appState.DATASET_ENABLED);
}

void ImGuiLayout::getImGui2DLayout(ApplicationState& appState)
{

}

void ImGuiLayout::getImGui3DLayout(ApplicationState& appState)
{



}

