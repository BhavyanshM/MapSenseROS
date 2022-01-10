//
// Created by quantum on 9/17/21.
//

#include <glm/gtc/type_ptr.hpp>
#include "Core/Timer.h"
#include "Core/Clay.h"
#include "opencv2/core/core.hpp"
#include "unistd.h"
#include "Scene/Mesh/TriangleMesh.h"
#include "Scene/Mesh/MeshTools.h"
#include "ImGui/ImGuiMenu.h"

#include "ApplicationLauncher.h"

#include "CameraParams.h"

namespace Clay
{
   ApplicationLauncher::ApplicationLauncher(int argc, char **argv) : Layer("Sandbox2D")
   {

      opt_fullscreen = true;
      dockspace_flags = ImGuiDockNodeFlags_None;
      window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;
      if (dockspace_flags & ImGuiDockNodeFlags_PassthruCentralNode)
         window_flags |= ImGuiWindowFlags_NoBackground;


      /* TODO: Instantiate the Information Processors */
      _networkManager = new NetworkManager(appState, &appUtils);
      _networkManager->init_ros_node(argc, argv, appState);

      _squareColor = glm::vec4(0.3, 0.9, 0.3, 1.0);

      _rootPCL = std::make_shared<Model>();
      Ref<Model> cameraGrandParent = std::make_shared<Model>(_rootPCL);
      Ref<Model> cameraParent = std::make_shared<Model>(cameraGrandParent);
      Ref<Model> cameraModel = std::make_shared<Model>(cameraParent);
      _cameraController = CameraController(1000.0f / 1000.0f, cameraModel);

      Clay::Ref<Clay::TriangleMesh> pose = std::make_shared<TriangleMesh>(glm::vec4(0.6f, 0.3f, 0.5f, 1.0f), _rootPCL);
      MeshTools::CoordinateAxes(pose);
      _poses.push_back(std::move(std::dynamic_pointer_cast<Model>(pose)));

   }

   void ApplicationLauncher::OnAttach()
   {
      CLAY_PROFILE_FUNCTION();
      FramebufferSpecification fbSpec;
      fbSpec.width = 1000;
      fbSpec.height = 1000;
      _frameBuffer = FrameBuffer::Create(fbSpec);
   }

   void ApplicationLauncher::OnDetach()
   {
      CLAY_PROFILE_FUNCTION();
      Layer::OnDetach();
   }

   void ApplicationLauncher::OnUpdate(Timestep ts)
   {
      CLAY_PROFILE_FUNCTION();

      MapsenseUpdate();

      if (_viewportFocused)
      {
         _cameraController.OnUpdate(ts);
      }

      Renderer::ResetStats();
      _frameBuffer->Bind();

      RenderCommand::SetClearColor({0.1f, 0.1f, 0.1f, 1});
      RenderCommand::Clear();

      Renderer::BeginScene(_cameraController.GetCamera());

      _rootPCL->Update();


      for(int i = 0; i<_models.size(); i++) Renderer::Submit(_models[i]);
      for(int i = 0; i<_poses.size(); i++)  Renderer::Submit(_poses[i]);

      Renderer::EndScene();
      _frameBuffer->Unbind();
   }

   void ApplicationLauncher::MapsenseUpdate()
   {
//      ROS_DEBUG("TickEvent: %d", count++);
      if (appState.ROS_ENABLED)
      {
         ROS_DEBUG("ROS Update: {}", appState.ROS_ENABLED);
         _networkManager->spin_ros_node();
         _networkManager->acceptMapsenseConfiguration(appState);
         _networkManager->receiverUpdate(appState);
         if (_networkManager->paramsAvailable)
         {
            _networkManager->paramsAvailable = false;
            appState.MERGE_DISTANCE_THRESHOLD = _networkManager->paramsMessage.mergeDistanceThreshold;
            appState.MERGE_ANGULAR_THRESHOLD = _networkManager->paramsMessage.mergeAngularThreshold;
         }

         ROS_DEBUG("ROS Update Completed");
      }
   }

   void ApplicationLauncher::OnEvent(Event& e)
   {
      _cameraController.OnEvent(e);
   }

   void ApplicationLauncher::OnImGuiRender()
   {
      CLAY_PROFILE_FUNCTION();

      if (opt_fullscreen)
      {
         const ImGuiViewport *viewport = ImGui::GetMainViewport();
         ImGui::SetNextWindowPos(viewport->WorkPos);
         ImGui::SetNextWindowSize(viewport->WorkSize);
         ImGui::SetNextWindowViewport(viewport->ID);
         ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
         ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
         window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
         window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;
      } else
      {
         dockspace_flags &= ~ImGuiDockNodeFlags_PassthruCentralNode;
      }

      ImGui::Begin("MapSense Dockspace", &dockspaceOpen, window_flags);
      if (opt_fullscreen)
         ImGui::PopStyleVar(2);

      // Submit the DockSpace
      ImGuiIO& io = ImGui::GetIO();
      if (io.ConfigFlags & ImGuiConfigFlags_DockingEnable)
      {
         ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
         ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), dockspace_flags);
      }

      if (ImGui::BeginMenuBar())
      {
         if (ImGui::BeginMenu("File"))
         {
            if (ImGui::MenuItem("Exit"))
               Application::Get().Close();

            ImGui::EndMenu();
         }
         ImGui::EndMenuBar();
      }

      /* Renderer ImGui Stats and Settings */
      ImGuiMenu::RendererOptions();

      /* Viewport Region */
      ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
      ImGui::Begin("Viewport");
      _viewportFocused = ImGui::IsWindowFocused();
      _viewportHovered = ImGui::IsWindowHovered();
      Application::Get().GetImGuiLayer()->BlockEvents(!_viewportFocused || !_viewportHovered);
      ImVec2 viewportPanelSize = ImGui::GetContentRegionAvail();

      if (_viewportSize != *((glm::vec2 *) &viewportPanelSize))
      {
         _viewportSize.x = viewportPanelSize.x;
         _viewportSize.y = viewportPanelSize.y;
         _frameBuffer->Resize((uint32_t) viewportPanelSize.x, (uint32_t) viewportPanelSize.y);
         _cameraController.OnResize(viewportPanelSize.x, viewportPanelSize.y);
      }
      uint32_t textureId = _frameBuffer->GetColorAttachment();
      ImGui::Image((void *) textureId, ImVec2{_viewportSize.x, _viewportSize.y}, ImVec2{0, 1}, ImVec2(1, 0));

      ImGui::End(); /* Viewport */
      ImGui::PopStyleVar();

      ImGui::Begin("MapSense Panel");
      if (ImGui::BeginTabBar("Configuration"))
      {
         if (ImGui::BeginTabItem("Application"))
         {
            ImGuiLayout::getImGuiAppLayout(appState);
            ImGui::EndTabItem();
         }
         ImGui::EndTabBar();
      }
      if (ImGui::BeginTabBar("Modules"))
      {

      }
      ImGui::End(); /* MapSense Panel */

      ImGui::Text("Models: %d", _models.size());

      ImGui::End(); /* Mapsense Dockspace */
   }

   void ApplicationLauncher::ImGuiUpdate(ApplicationState& appState)
   {
      ImGui::Text("Input:%d,%d Patch:%d,%d Level:%d", appState.INPUT_HEIGHT, appState.INPUT_WIDTH, appState.PATCH_HEIGHT, appState.PATCH_WIDTH,
                  appState.KERNEL_SLIDER_LEVEL);
      ImGui::Checkbox("Generate Regions", &appState.GENERATE_REGIONS);
      ImGui::Checkbox("Filter", &appState.FILTER_SELECTED);
      ImGui::Checkbox("Early Gaussian", &appState.EARLY_GAUSSIAN_BLUR);
      ImGui::SliderInt("Gaussian Size", &appState.GAUSSIAN_SIZE, 1, 4);
      ImGui::SliderInt("Gaussian Sigma", &appState.GAUSSIAN_SIGMA, 1, 20);
      ImGui::SliderFloat("Distance Threshold", &appState.MERGE_DISTANCE_THRESHOLD, 0.0f, 0.1f);
      ImGui::SliderFloat("Angular Threshold", &appState.MERGE_ANGULAR_THRESHOLD, 0.0f, 1.0f);
      ImGui::Checkbox("Components", &appState.SHOW_REGION_COMPONENTS);
      ImGui::Checkbox("Boundary", &appState.SHOW_BOUNDARIES);
      ImGui::Checkbox("Internal", &appState.SHOW_PATCHES);
      if (ImGui::Button("Hide Display"))
      {
         cv::destroyAllWindows();
      }
      ImGui::SliderFloat("Display Window Size", &appState.DISPLAY_WINDOW_SIZE, 0.1, 5.0);
      ImGui::SliderFloat("Depth Brightness", &appState.DEPTH_BRIGHTNESS, 1.0, 100.0);
      ImGui::Checkbox("Visual Debug", &appState.VISUAL_DEBUG);
      ImGui::SliderInt("Visual Debug Delay", &appState.VISUAL_DEBUG_DELAY, 1, 100);
      ImGui::Checkbox("Show Edges", &appState.SHOW_REGION_EDGES);
      ImGui::SliderInt("Skip Edges", &appState.NUM_SKIP_EDGES, 1, 20);
      ImGui::SliderFloat("Magnum Patch Scale", &appState.MAGNUM_PATCH_SCALE, 0.001f, 0.04f);
   }
}
