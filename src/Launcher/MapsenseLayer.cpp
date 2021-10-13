//
// Created by quantum on 9/17/21.
//

#include <glm/gtc/type_ptr.hpp>
#include "MapsenseLayer.h"
#include "Core/Timer.h"
#include "Core/Clay.h"
#include "opencv2/core/core.hpp"
#include "unistd.h"

namespace Clay
{
   MapsenseLayer::MapsenseLayer(int argc, char **argv) : Layer("Sandbox2D")
   {
      opt_fullscreen = true;
      dockspace_flags = ImGuiDockNodeFlags_None;
      window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;
      if (dockspace_flags & ImGuiDockNodeFlags_PassthruCentralNode)
         window_flags |= ImGuiWindowFlags_NoBackground;

      /* TODO: Instantiate the Information Processors */
      _openCLManager = new OpenCLManager();
      _networkManager = new NetworkManager(appState, &appUtils);
      _networkManager->init_ros_node(argc, argv, appState);

      _regionCalculator = new PlanarRegionCalculator(argc, argv, _networkManager, appState);
      _regionCalculator->setOpenCLManager(_openCLManager);

      _icp = new IterativeClosestPoint();
      _icp->SetOpenCLManager(_openCLManager);

      _keypointDetector = new KeypointDetector(argc, argv, _networkManager, appState);

      _slamModule = new SLAMModule(argc, argv, _networkManager);


      _squareColor = glm::vec4(0.3, 0.9, 0.3, 1.0);

      _rootPCL = std::make_shared<Model>();
      Ref<Model> cameraGrandParent = std::make_shared<Model>(_rootPCL);
      Ref<Model> cameraParent = std::make_shared<Model>(cameraGrandParent);
      Ref<Model> cameraModel = std::make_shared<Model>(cameraParent);
      _cameraController = CameraController(1000.0f / 1000.0f, cameraModel);

      Ref<PointCloud> firstCloud = std::make_shared<PointCloud>(ros::package::getPath("map_sense") + "/Extras/Clouds/Scan_4", glm::vec4(0.7f, 0.4f, 0.5f, 1.0f), _rootPCL);
      Ref<PointCloud> cloud = std::make_shared<PointCloud>(ros::package::getPath("map_sense") + "/Extras/Clouds/Scan_8", glm::vec4(0.1f, 0.2f, 0.8f, 1.0f), firstCloud);

//      Ref<PointCloud> firstCloud = std::make_shared<PointCloud>(std::string(ASSETS_PATH) + "Meshes/bunny.pcd", glm::vec4(0.7f, 0.4f, 0.5f, 1.0f), _rootPCL);
//      Ref<PointCloud> cloud = std::make_shared<PointCloud>(std::string(ASSETS_PATH) + "Meshes/bunny.pcd", glm::vec4(0.1f, 0.2f, 0.8f, 1.0f), firstCloud);

      cloud->RotateLocalX(-0.3f);
      cloud->RotateLocalY(0.4f);
      cloud->TranslateLocal({0.3f, 0.5f, -0.73f});

      _models.emplace_back(std::dynamic_pointer_cast<Model>(firstCloud));
      _models.emplace_back(std::dynamic_pointer_cast<Model>(cloud));

//      _pclReceiver = ((PointCloudReceiver*)_networkManager->receivers[2]);
//      Ref<PointCloud> pclCloud = _pclReceiver->GetRenderable();
//      _models.emplace_back(std::dynamic_pointer_cast<Model>(pclCloud));
   }

   void MapsenseLayer::OnAttach()
   {
      CLAY_PROFILE_FUNCTION();

      cv::Mat image = FileManager::ReadImage("/Github_Images/Combined_FirstPage_v2.jpg");
      _texture = Texture2D::Create();
      _texture->LoadImage(image.data, image.cols, image.rows, image.channels());

      cv::Mat imageCheckerboard = FileManager::ReadImage("/Github_Images/Checkerboard.png");
      _checkerTexture = Texture2D::Create();
      _checkerTexture->LoadImage(imageCheckerboard.data, imageCheckerboard.cols, imageCheckerboard.rows, imageCheckerboard.channels());

      FramebufferSpecification fbSpec;
      fbSpec.width = 1000;
      fbSpec.height = 1000;
      _frameBuffer = FrameBuffer::Create(fbSpec);
   }

   void MapsenseLayer::OnDetach()
   {
      CLAY_PROFILE_FUNCTION();
      Layer::OnDetach();
   }

   void MapsenseLayer::OnUpdate(Timestep ts)
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
      if(_models.size() > 1) _models[1]->SetColor(_squareColor);
      for(Ref<Model> model : _models)
      {
         Renderer::SubmitPoints(model);
      }

      Renderer::EndScene();
      _frameBuffer->Unbind();

   }

   void MapsenseLayer::MapsenseUpdate()
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

         _regionCalculator->generateRegionsFromDepth(appState);
         _regionCalculator->publish();
         _regionCalculator->render();

         //      _keypointDetector->update(appState);

         if (_regionCalculator->planarRegionList.size() > 0 && appState.ROS_ENABLED && _slamModule->_mapper.SLAM_ENABLED)
         {
            PlanarRegion::PrintRegionList(_regionCalculator->planarRegionList, "Initial Planar Regions");
            _slamModule->setLatestRegionsToZUp(_regionCalculator->planarRegionList);
            _slamModule->slamUpdate();

            /* TODO: Publish the latest optimized pose from Factor Graph SLAM. */

            //         vector<RigidBodyTransform> sensorTransforms = _slamModule->_mapper.poses;
            //         if (sensorTransforms.size() > 0 && _slamModule->enabled)
            //         {
            //            _networkManager->publishSLAMPose(sensorTransforms.rbegin()[0]);
            //            printf("After SLAM Publisher.\n");
            //         }
         }

         appUtils.clearDebug();



      }
   }

   void MapsenseLayer::OnEvent(Event& e)
   {
      _cameraController.OnEvent(e);
   }

   void MapsenseLayer::OnImGuiRender()
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
      ImGui::Begin("Renderer");
      ImGui::ColorEdit3("Square Color", glm::value_ptr(_squareColor));
      auto stats = Renderer::GetStats();
      ImGui::Text("Renderer Stats:");
      ImGui::Text("Draw Calls: %d", stats.DrawCalls);
      ImGui::Text("Quad Count: %d", stats.TriangleCount);
      ImGui::Text("Vertices: %d", stats.GetTotalVertexCount());
      ImGui::Text("Indices: %d", stats.GetTotalIndexCount());
      ImGui::End();

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
      ImGui::End();
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
         if (ImGui::BeginTabItem("Planar Regions"))
         {
            /* Display 2D */
            _regionCalculator->ImGuiUpdate(appState);
            ImGui::EndTabItem();
         }
         if (ImGui::BeginTabItem("SLAM"))
         {
            _slamModule->ImGuiUpdate();
            ImGui::EndTabItem();
         }
         if (ImGui::BeginTabItem("Network"))
         {
            _networkManager->ImGuiUpdate();
            ImGui::EndTabItem();
         }
         if (ImGui::BeginTabItem("Extras"))
         {
            /* Display 3D */
            ImGui::SameLine(180);

            /* Beta Features */
            if (ImGui::Button("Save All"))
            {
               AppUtils::capture_data(ros::package::getPath("map_sense"), "/Extras/Images/Capture", _regionCalculator->inputDepth,
                                      _regionCalculator->inputColor, _regionCalculator->filteredDepth, _regionCalculator->_mapFrameProcessor.debug, appState,
                                      _regionCalculator->planarRegionList);
            }
            if (ImGui::Button("Save Regions"))
            {
               GeomTools::saveRegions(_regionCalculator->planarRegionList, ros::package::getPath("map_sense") + "/Extras/Regions/" +
                                                                           string(4 - to_string(frameId).length(), '0').append(to_string(frameId)) + ".txt");
               frameId++;
            }

            if (ImGui::Button("Configure Memory"))
               AppUtils::checkMemoryLimits();

            ImGui::Checkbox("Stereo-Left", &appState.SHOW_STEREO_LEFT);
            ImGui::Checkbox("Stereo-Right", &appState.SHOW_STEREO_RIGHT);

            if (ImGui::Button("Log OpenCV Build Info"))
            {
               cout << cv::getBuildInformation() << endl;
            }

            ImGui::EndTabItem();
         }
         ImGui::EndTabBar();
      }
      ImGui::End();

      ImGui::Begin("LIDAR-ICP Panel");
      if(ImGui::Button("Load Next Scan"))
      {
//         _models.clear();
         _scanCount+=4;
         std::string filename = ros::package::getPath("map_sense") + "/Extras/Clouds/Scan_" + std::to_string(_scanCount);
         CLAY_LOG_INFO("Loading Scan From: {}", filename);

         Ref<PointCloud> cloud = std::make_shared<PointCloud>(
               filename,glm::vec4(0.5f, 0.32f, 0.8f, 1.0f), _rootPCL);
         _models.emplace_back(std::dynamic_pointer_cast<Model>(cloud));
      }
      ImGui::Text("Models: %d", _models.size());
      if(ImGui::Button("Calculate ICP"))
      {
         Eigen::Matrix4f transformOne, transformTwo;
         glm::mat4 invTransformOne = glm::inverse(_models[0]->GetTransformToParent());
         glm::mat4 invTransformTwo = glm::inverse(_models[1]->GetTransformToParent());
         for (int i = 0; i < 4; ++i)
         {
            for (int j = 0; j < 4; ++j)
            {
               transformOne(i,j) = invTransformOne[j][i];
               transformTwo(i,j) = invTransformTwo[j][i];
            }
         }

         std::cout << "Transform Before Conversion:" << transformTwo << std::endl;
         std::cout << "Transform After Conversion:" << glm::to_string(_models[1]->GetTransformToParent()) << std::endl;

         // Calculate ICP based Pointcloud Alignment.
         Eigen::Matrix4f transformEigen = _icp->CalculateAlignment(_models[0]->GetMesh()->_vertices, transformOne, _models[1]->GetMesh()->_vertices, transformTwo);

         glm::mat4 transform;
         for (int i = 0; i < 4; ++i)
         {
            for (int j = 0; j < 4; ++j)
            {
               transform[j][i] = transformEigen(i, j);
            }
         }
         _models[1]->TransformLocal(transform);
      }
      ImGui::End();

      ImGui::End();
   }
}
