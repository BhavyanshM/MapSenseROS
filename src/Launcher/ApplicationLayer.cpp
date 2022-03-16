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

#include "ApplicationLayer.h"
#include "CameraParams.h"
#include "implot.h"

namespace Clay
{
   ApplicationLayer::~ApplicationLayer() noexcept
   {
      ImPlot::DestroyContext();
   }

   ApplicationLayer::ApplicationLayer(int argc, char **argv) : Layer("Sandbox2D")
   {

      std::vector<std::string> args(argv, argv + argc);
      for (int i = 0; i < argc; i++)
      {
         if (args[i] == "--export")
         {
            printf("Setting EXPORT_REGIONS: true\n");
            appState.EXPORT_REGIONS = true;
         }
         if (args[i] == "--kernel-level")
         {
            printf("Setting KERNEL_LEVEL_SLIDER: %d\n", stoi(args[i + 1]));
            appState.KERNEL_SLIDER_LEVEL = stoi(args[i + 1]);
         }
         if (args[i] == "--stereo-driver")
         {
            printf("Setting STEREO_DRIVER: true\n");
            appState.STEREO_DRIVER = true;
         }
         if (args[i] == "--depth-aligned")
         {
            printf("Setting DEPTH_ALIGNED: true\n");
            appState.DEPTH_ALIGNED = true;
         }
         if (args[i] == "--camera-name")
         {
            printf("Setting TOPIC_CAMERA_NAME: %s\n", args[i + 1].c_str());
            appState.TOPIC_CAMERA_NAME = args[i + 1];
         }
      }

      opt_fullscreen = true;
      dockspace_flags = ImGuiDockNodeFlags_None;
      window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;
      if (dockspace_flags & ImGuiDockNodeFlags_PassthruCentralNode)
         window_flags |= ImGuiWindowFlags_NoBackground;

      ImPlot::CreateContext();

      /* TODO: Instantiate the Information Processors */
      _openCLManager = new OpenCLManager(ros::package::getPath("map_sense"));
      _networkManager = new NetworkManager(appState, &appUtils);
      _networkManager->init_ros_node(argc, argv, appState);

      _squareColor = glm::vec4(0.3, 0.9, 0.3, 1.0);

      _rootModel = std::make_shared<Model>();
      Ref<Model> cameraGrandParent = std::make_shared<Model>(_rootModel);
      Ref<Model> cameraParent = std::make_shared<Model>(cameraGrandParent);
      Ref<Model> cameraModel = std::make_shared<Model>(cameraParent);
      _cameraController = CameraController(1000.0f / 1000.0f, cameraModel);

      printf("Here\n");
   }

   void ApplicationLayer::OnAttach()
   {
      CLAY_PROFILE_FUNCTION();
      FramebufferSpecification fbSpec;
      fbSpec.width = 1000;
      fbSpec.height = 1000;
      _frameBuffer = FrameBuffer::Create(fbSpec);
   }

   void ApplicationLayer::OnDetach()
   {
      CLAY_PROFILE_FUNCTION();
      Layer::OnDetach();
   }

   void ApplicationLayer::OnUpdate(Timestep ts)
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
      RenderCommand::SetLineWidth(_lineWidth);
      RenderCommand::SetPointSize(_pointSize);
      RenderCommand::Clear();

      Renderer::BeginScene(_cameraController.GetCamera());

      _rootModel->Update();

      for (int i = 0; i < _models.size(); i++)
         Renderer::Submit(_models[i]);
      for (int i = 0; i < mesher.GetModels().size(); i++)
         Renderer::Submit(mesher.GetModels()[i]);
      for (int i = 0; i < mesher.GetPoses().size(); i++)
         Renderer::Submit(mesher.GetPoses()[i]);
      for (int i = 0; i < mesher.GetLines().size(); i++)
         Renderer::Submit(mesher.GetLines()[i]);

      Renderer::EndScene();
      _frameBuffer->Unbind();
   }

   void ApplicationLayer::OnEvent(Event& e)
   {
      _cameraController.OnEvent(e);
   }

   void ApplicationLayer::OnImGuiRender()
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

         if (ImGui::BeginTabBar("Modules"))
         {
            ImGuiUpdate(appState);
            ImGui::EndTabBar();
         }

         ImGui::EndTabBar();
      }


      ImGui::End(); /* MapSense Panel */
      ImGui::End(); /* Mapsense Dockspace */

   }
}
