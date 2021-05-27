#include "ImGuiLayout.h"

void ImGuiLayout::getImGuiROSLayout(ApplicationState& appState)
{
   ImGui::Checkbox("ROS Enabled", &appState.ROS_ENABLED);
   ImGui::Checkbox("Generate Regions", &appState.GENERATE_REGIONS);
}

void ImGuiLayout::getImGuiParamsLayout(ApplicationState& appState)
{
   ImGui::Text("Input:%d,%d Patch:%d,%d Level:%d", appState.INPUT_HEIGHT, appState.INPUT_WIDTH, appState.PATCH_HEIGHT, appState.PATCH_WIDTH,
               appState.KERNEL_SLIDER_LEVEL);
   ImGui::Checkbox("Filter", &appState.FILTER_SELECTED);
   ImGui::Checkbox("Early Gaussian", &appState.EARLY_GAUSSIAN_BLUR);
   ImGui::SliderInt("Gaussian Size", &appState.GAUSSIAN_SIZE, 1, 4);
   ImGui::SliderFloat("Gaussian Sigma", &appState.GAUSSIAN_SIGMA, 1.0f, 20.0f);
   ImGui::SliderFloat("Merge Distance Threshold", &appState.MERGE_DISTANCE_THRESHOLD, 0.0f, 0.1f);
   ImGui::SliderFloat("Merge Angular Threshold", &appState.MERGE_ANGULAR_THRESHOLD, 0.0f, 1.0f);
   ImGui::SliderFloat("Region Growth Factor", &appState.REGION_GROWTH_FACTOR, 0.001, 0.1);
   
   //    ImGui::SliderInt("Region Boundary Diff", &appState.REGION_BOUNDARY_DIFF, 10, 40);
   //    ImGui::SliderInt("Region Min Patches", &appState.REGION_MIN_PATCHES, 4, 100);

   //    ImGui::SliderFloat("Filter Disparity Threshold", &appState.FILTER_DISPARITY_THRESHOLD, 1000, 4000);
   //    ImGui::SliderInt("Kernel Level", &appState.KERNEL_SLIDER_LEVEL, 2, 10);
   //    ImGui::SliderInt("Filter Size", &appState.FILTER_KERNEL_SIZE, 2, 10);
   //    appState.update();
}

void ImGuiLayout::getImGui2DLayout(ApplicationState& appState)
{
   ImGui::Checkbox("Input Color", &appState.SHOW_INPUT_COLOR);
   ImGui::Checkbox("Input Depth", &appState.SHOW_INPUT_DEPTH);
   ImGui::SameLine(180);
   ImGui::Checkbox("Graph", &appState.SHOW_GRAPH);
   ImGui::Checkbox("Filtered Depth", &appState.SHOW_FILTERED_DEPTH);
   ImGui::Checkbox("Region Components", &appState.SHOW_REGION_COMPONENTS);
   ImGui::Checkbox("Boundary", &appState.SHOW_BOUNDARIES);
   ImGui::Checkbox("Internal", &appState.SHOW_PATCHES);

   if (ImGui::Button("Hide Display"))
   {
      destroyAllWindows();
      destroyAllWindows();
   }
   ImGui::SliderFloat("Display Window Size", &appState.DISPLAY_WINDOW_SIZE, 0.1, 5.0);
   ImGui::SliderFloat("Depth Brightness", &appState.DEPTH_BRIGHTNESS, 1.0, 100.0);

   ImGui::Checkbox("Visual Debug", &appState.VISUAL_DEBUG);
   ImGui::SliderInt("Visual Debug Delay", &appState.VISUAL_DEBUG_DELAY, 1, 100);
}

void ImGuiLayout::getImGui3DLayout(ApplicationState& appState, Magnum::Color4& color)
{
   if (ImGui::ColorEdit3("Color", color.data()))
   {
      Magnum::GL::Renderer::setClearColor(color);
   }
   ImGui::Checkbox("Show Edges", &appState.SHOW_REGION_EDGES);
   ImGui::SliderInt("Skip Edges", &appState.NUM_SKIP_EDGES, 1, 20);
   ImGui::Text("Time:%.3f ms FPS:%.1f", 1000.0 / Magnum::Double(ImGui::GetIO().Framerate), Magnum::Double(ImGui::GetIO().Framerate));
   ImGui::SliderFloat("Magnum Patch Scale", &appState.MAGNUM_PATCH_SCALE, 0.001f, 0.04f);

}

