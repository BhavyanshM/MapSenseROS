#include "ImGuiLayout.h"

void ImGuiLayout::getImGuiAppLayout(ApplicationState& appState)
{
   ImGui::Checkbox("ROS Enabled", &appState.ROS_ENABLED);
   Magnum::Color4 color;
   if (ImGui::ColorEdit3("Color", color.data()))
   {
      Magnum::GL::Renderer::setClearColor(color);
   }
   ImGui::Text("Time:%.3f ms FPS:%.1f", 1000.0 / Magnum::Double(ImGui::GetIO().Framerate), Magnum::Double(ImGui::GetIO().Framerate));
}



void ImGuiLayout::getImGui2DLayout(ApplicationState& appState)
{

}

void ImGuiLayout::getImGui3DLayout(ApplicationState& appState, Magnum::Color4& color)
{



}

