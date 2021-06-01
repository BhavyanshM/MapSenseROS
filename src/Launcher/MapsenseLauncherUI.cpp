#include "MapsenseLauncherUI.h"

MyApplication::MyApplication(const Arguments& arguments) : Magnum::Platform::Application{arguments,
                                                                                         Configuration{}.setTitle("Magnum ImGui Application").setSize(
                                                                                               Magnum::Vector2i(1024, 768)).setWindowFlags(
                                                                                               Configuration::WindowFlag::Resizable)}
{

   _imgui = Magnum::ImGuiIntegration::Context(Magnum::Vector2{windowSize()} / dpiScaling(), windowSize(), framebufferSize());

   /* Set up proper blending to be used by ImGui. There's a great chance
      you'll need this exact behavior for the rest of your scene. If not, set
      this only for the drawFrame() call. */
   Magnum::GL::Renderer::setBlendEquation(Magnum::GL::Renderer::BlendEquation::Add, Magnum::GL::Renderer::BlendEquation::Add);
   Magnum::GL::Renderer::setBlendFunction(Magnum::GL::Renderer::BlendFunction::SourceAlpha, Magnum::GL::Renderer::BlendFunction::OneMinusSourceAlpha);

   ImGuiIO& io = ImGui::GetIO();
   io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

   /* TODO: Check that the appropriate flags for renderer are set*/
   Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::DepthTest);
   // Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::FaceCulling);

   /* TODO: Configure Camera in Scene Graph*/
   _camGrandParent = new Object3D{&_scene};
   _sensor = new Object3D{&_scene};
   _camParent = new Object3D{_camGrandParent};
   _camObject = new Object3D{_camParent};
   _camera = new Magnum::SceneGraph::Camera3D(*_camObject);
   _camera->setProjectionMatrix(Magnum::Matrix4::perspectiveProjection(35.0_degf, 1.33f, 0.001f, 100.0f));
   _sensor->transformLocal(Magnum::Matrix4::rotationX(Magnum::Rad{90.0_degf}));

   /* TODO: Prepare your objects here and add them to the scene */
   _sensorAxes = new Object3D{_sensor};
   _sensorAxes->scale({0.1, 0.1, 0.1});
   new RedCubeDrawable{*_sensorAxes, &_drawables, Magnum::Primitives::axis3D(), {0.5, 0.1f, 0.1f}};

   _camObject->translate({0, 0, 10.0f});
   _sensor->transformLocal(Magnum::Matrix4::rotationX(Magnum::Rad{90.0_degf}));

   _camOriginCube = new Object3D{_camGrandParent};
   _camOriginCube->scale({0.01f, 0.01f, 0.01f});
   new RedCubeDrawable{*_camOriginCube, &_drawables, Magnum::Primitives::cubeSolid(), {0.2, 0.0f, 0.3f}};

   // setMinimalLoopPeriod(32); /* Needs to be less than 30-32 milliseconds for real-time performance */

   /* --------------------------------------- For Testing Purposes Only --------------------------------------------*/
   //    generate_patches();

}

void MyApplication::init(int argc, char **argv)
{
   /* TODO: Instantiate the Information Processors */
   _networkManager = new NetworkManager(appState, &appUtils);
   _networkManager->init_ros_node(argc, argv, appState);

   _regionCalculator = new PlanarRegionCalculator(appState);
   _regionCalculator->initOpenCL(appState);

   _slamModule = new SLAMModule(argc, argv);

   ROS_DEBUG("Application Initialized Successfully");
}

void MyApplication::tickEvent()
{
   ROS_INFO("TickEvent: %d", count++);

   _regionCalculator->render();

   if (appState.ROS_ENABLED)
   {
      _networkManager->spin_ros_node();
      _networkManager->receiverUpdate(appState);
      if (_networkManager->paramsAvailable)
      {
         _networkManager->paramsAvailable = false;
         appState.MERGE_DISTANCE_THRESHOLD = _networkManager->paramsMessage.mergeDistanceThreshold;
         appState.MERGE_ANGULAR_THRESHOLD = _networkManager->paramsMessage.mergeAngularThreshold;
      }
      //      _networkManager->load_next_frame(_regionCalculator->inputDepth, _regionCalculator->inputColor, _regionCalculator->inputTimestamp, appState);
      ImageReceiver *depthReceiver = ((ImageReceiver *) _networkManager->receivers[0]);
      ROS_DEBUG("Loading Data into Depth Receiver");
      depthReceiver->getData(_regionCalculator->inputDepth, appState, _regionCalculator->inputTimestamp);
      if (depthReceiver->cameraInfoSet && appState.GENERATE_REGIONS)
      {
         _regionCalculator->generateRegions(_networkManager, appState);
         if (appState.EXPORT_REGIONS)
         {
            if (frameId % 10 == 0)
            {
               AppUtils::write_regions(_regionCalculator->planarRegionList, ros::package::getPath("map_sense") + "/Extras/Regions/" +
                                                                            string(4 - to_string(frameId).length(), '0').append(to_string(frameId)) + ".txt");
            }
            frameId++;
         }
      }

//      ROS_INFO("Latest Regions for SLAM: %d", _regionCalculator->planarRegionList.size());
//      if (_regionCalculator->planarRegionList.size() > 0)
//      {
//         _slamModule->slamUpdate(_regionCalculator->planarRegionList);
//         printf("After SLAM Update.\n");
//         vector<RigidBodyTransform> sensorTransforms = _slamModule->_mapper.poses;
//         if (sensorTransforms.size() > 0)
//         {
//            _networkManager->publishSLAMPose(sensorTransforms.rbegin()[0]);
//            printf("After SLAM Publisher.\n");
//         }
//      }
//      ROS_INFO("SLAM Pose Published.");

      if (appState.STEREO_DRIVER)
      {
         _networkManager->load_next_stereo_frame(_regionCalculator->inputStereoLeft, _regionCalculator->inputStereoRight, appState);
         //         AppUtils::displayDebugOutput(_regionCalculator->inputStereoLeft, appState);
      }
      if (appState.SHOW_REGION_EDGES)
      {
         MeshGenerator::clearMesh(regionEdges);
         draw_regions();
      }
   }
}

void MyApplication::viewportEvent(ViewportEvent& event)
{
   Magnum::GL::defaultFramebuffer.setViewport({{}, event.framebufferSize()});
   _imgui.relayout(Magnum::Vector2{event.windowSize()} / event.dpiScaling(), event.windowSize(), event.framebufferSize());
}

void MyApplication::mousePressEvent(MouseEvent& event)
{
   if (_imgui.handleMousePressEvent(event))
   {
      return;
   }
}

void MyApplication::mouseReleaseEvent(MouseEvent& event)
{
   if (_imgui.handleMouseReleaseEvent(event))
      return;
}

void MyApplication::mouseMoveEvent(MouseMoveEvent& event)
{
   if (_imgui.handleMouseMoveEvent(event))
   {
      return;
   }
   if ((event.buttons() & MouseMoveEvent::Button::Left))
   {
      Magnum::Vector2 delta = 4.0f * Magnum::Vector2{event.relativePosition()} / Magnum::Vector2{windowSize()};
      _camGrandParent->transformLocal(Magnum::Matrix4::rotationY(-Magnum::Rad{delta.x()}));
      _camOriginCube->transformLocal(_scene.absoluteTransformation());
      _camParent->transformLocal(Magnum::Matrix4::rotationX(-Magnum::Rad{delta.y()}));
   }
   if ((event.buttons() & MouseMoveEvent::Button::Right))
   {
      Magnum::Vector2 delta = 4.0f * Magnum::Vector2{event.relativePosition()} / Magnum::Vector2{windowSize()};
      _camGrandParent->translateLocal({-0.5f * delta.x(), 0, -0.5f * delta.y()});
      _camOriginCube->translateLocal({-0.5f * delta.x(), 0, -0.5f * delta.y()});
   }
   if ((event.buttons() & MouseMoveEvent::Button::Middle))
   {
      Magnum::Vector2 delta = 4.0f * Magnum::Vector2{event.relativePosition()} / Magnum::Vector2{windowSize()};
      _camGrandParent->translateLocal({0, 0.8f * delta.y(), 0});
      _camOriginCube->translateLocal({0, 0.8f * delta.y(), 0});
   }
   event.setAccepted();
}

void MyApplication::mouseScrollEvent(MouseScrollEvent& event)
{
   if (_imgui.handleMouseScrollEvent(event))
   {
      /* Prevent scrolling the page */
      return;
   }
   Magnum::Vector2 delta = Magnum::Vector2{event.offset()};
   _camObject->translate({0, 0, -0.5f * delta.y()});
   event.setAccepted();
}

void MyApplication::draw_patches()
{
   for (int i = 0; i < _regionCalculator->output.getRegionOutput().rows; i++)
   {
      for (int j = 0; j < _regionCalculator->output.getRegionOutput().cols; j++)
      {
         uint8_t edges = _regionCalculator->output.getPatchData().at<uint8_t>(i, j);
         if (edges == 255)
         {
            Vec6f patch = _regionCalculator->output.getRegionOutput().at<Vec6f>(i, j);
            // cout << patch << endl;
            Magnum::Vector3 up = {0, 0, 1};
            Magnum::Vector3 dir = {0.01f * patch[0], 0.01f * patch[1], 0.01f * patch[2]};
            Magnum::Vector3 axis = Magnum::Math::cross(up, dir).normalized();
            Magnum::Rad angle = Magnum::Math::acos(Magnum::Math::dot(up, dir) / (up.length() * dir.length()));

            Object3D& plane = _sensor->addChild<Object3D>();
            planePatches.emplace_back(&plane);
            plane.scale({appState.MAGNUM_PATCH_SCALE, appState.MAGNUM_PATCH_SCALE, appState.MAGNUM_PATCH_SCALE});
            plane.translate({patch[3], patch[4], patch[5]});
            // plane.transformLocal(Magnum::Matrix4::rotationX(-Magnum::Rad{180.0_degf}));
            if (!isnan(axis.x()) && !isnan(axis.y()) && !isnan(axis.z()))
            {
               Magnum::Quaternion quat = Magnum::Quaternion::rotation(angle, axis);
               plane.transformLocal(Magnum::Matrix4(quat.toMatrix()));
            }
            new RedCubeDrawable{plane, &_drawables, Magnum::Primitives::planeSolid(), {0.4, 0.4f, 0.6f}};
         }
      }
   }
}

void MyApplication::draw_regions()
{
   auto start = high_resolution_clock::now();
   for (int i = 0; i < _regionCalculator->planarRegionList.size(); i++)
   {
      shared_ptr<PlanarRegion> planarRegion = _regionCalculator->planarRegionList[i];
      vector<Eigen::Vector3f> vertices = planarRegion->getVertices();
      for (int j = appState.NUM_SKIP_EDGES; j < vertices.size(); j += appState.NUM_SKIP_EDGES)
      {
         Object3D& edge = _sensor->addChild<Object3D>();
         Eigen::Vector3f prevPoint = vertices[j - appState.NUM_SKIP_EDGES];
         Eigen::Vector3f curPoint = vertices[j];
         regionEdges.emplace_back(&edge);
         new RedCubeDrawable{edge, &_drawables,
                             Magnum::Primitives::line3D({prevPoint.x(), prevPoint.y(), prevPoint.z()}, {curPoint.x(), curPoint.y(), curPoint.z()}),
                             {(planarRegion->getId() * 123 % 255) / 255.0f, (planarRegion->getId() * 161 % 255) / 255.0f,
                              (planarRegion->getId() * 113 % 255) / 255.0f}};
      }
   }
   auto end = high_resolution_clock::now();
   auto duration = duration_cast<microseconds>(end - start).count();
   ROS_DEBUG("Visualization Took: %.2f ms", duration / (float) 1000);
}

void MyApplication::generate_patches()
{
   _regionCalculator->generateRegions(_networkManager, appState);
   draw_patches();
}

void MyApplication::drawEvent()
{
   Magnum::GL::defaultFramebuffer.clear(Magnum::GL::FramebufferClear::Color | Magnum::GL::FramebufferClear::Depth);
   _camera->draw(_drawables);

   _imgui.newFrame();

   ImGui::Text("MapSense");


   /* Use for docking. */

   //   static ImGuiID dockspaceID = 0;
   //   bool active = true;
   //   if (ImGui::Begin("Master Window", &active))
   //   {
   //      ImGui::TextUnformatted("DockSpace below");
   //   }
   //   if (active)
   //   {
   //      // Declare Central dockspace
   //      dockspaceID = ImGui::GetID("HUB_DockSpace");
   //      ImGui::DockSpace(dockspaceID, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_None | ImGuiDockNodeFlags_PassthruCentralNode/*|ImGuiDockNodeFlags_NoResize*/);
   //   }
   //   ImGui::End();
   //
   //   ImGui::SetNextWindowDockID(dockspaceID, ImGuiCond_FirstUseEver);
   //   if (ImGui::Begin("Dockable Window"))
   //   {
   //      ImGui::TextUnformatted("Test");
   //   }
   //   ImGui::End();

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
         if (ImGui::Button("Generate Patches"))
         {
            MeshGenerator::clearMesh(planePatches);
            generate_patches();
         }
         ImGui::SameLine(180);
         if (ImGui::Button("Clear Patches"))
            MeshGenerator::clearMesh(planePatches);

         /* Beta Features */
         if (ImGui::Button("Save All"))
         {
            AppUtils::capture_data(ros::package::getPath("map_sense"), "/Extras/Images/Capture", _regionCalculator->inputDepth, _regionCalculator->inputColor,
                                   _regionCalculator->filteredDepth, _regionCalculator->mapFrameProcessor.debug, appState, _regionCalculator->planarRegionList);
         }
         if (ImGui::Button("Save Regions"))
         {
            AppUtils::write_regions(_regionCalculator->planarRegionList, ros::package::getPath("map_sense") + "/Extras/Regions/" +
                                                                         string(4 - to_string(frameId).length(), '0').append(to_string(frameId)) + ".txt");
            frameId++;
         }

         if (ImGui::Button("Configure Memory"))
            AppUtils::checkMemoryLimits();

         ImGui::Checkbox("Stereo-Left", &appState.SHOW_STEREO_LEFT);
         ImGui::Checkbox("Stereo-Right", &appState.SHOW_STEREO_RIGHT);

         if (ImGui::Button("Log OpenCV Build Info"))
         {
            cout << getBuildInformation() << endl;
         }

         ImGui::EndTabItem();
      }
      ImGui::EndTabBar();
   }


   /* Update application cursor */
   _imgui.updateApplicationCursor(*this);

   Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::Blending);
   Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::ScissorTest);
   Magnum::GL::Renderer::disable(Magnum::GL::Renderer::Feature::DepthTest);
   Magnum::GL::Renderer::disable(Magnum::GL::Renderer::Feature::FaceCulling);

   _imgui.drawFrame();

   Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::DepthTest);
   // Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::FaceCulling);
   Magnum::GL::Renderer::disable(Magnum::GL::Renderer::Feature::ScissorTest);
   Magnum::GL::Renderer::disable(Magnum::GL::Renderer::Feature::Blending);

   swapBuffers();
   redraw();
}

void MyApplication::exitEvent(ExitEvent& event)
{
   _networkManager->camLeft->release();
   _networkManager->camRight->release();
   event.setAccepted(true);
}

int main(int argc, char **argv)
{
   MyApplication app({argc, argv});
   std::vector<std::string> args(argv, argv + argc);

   for (int i = 0; i < argc; i++)
   {
      if (args[i] == "--export")
      {
         printf("Setting EXPORT_REGIONS: true\n");
         app.appState.EXPORT_REGIONS = true;
      }
      if (args[i] == "--kernel-level")
      {
         printf("Setting KERNEL_LEVEL_SLIDER: %d\n", stoi(args[i + 1]));
         app.appState.KERNEL_SLIDER_LEVEL = stoi(args[i + 1]);
      }
      if (args[i] == "--stereo-driver")
      {
         printf("Setting STEREO_DRIVER: true\n");
         app.appState.STEREO_DRIVER = true;
      }
      if (args[i] == "--depth-aligned")
      {
         printf("Setting DEPTH_ALIGNED: true\n");
         app.appState.DEPTH_ALIGNED = true;
      }
      if (args[i] == "--camera-name")
      {
         printf("Setting TOPIC_CAMERA_NAME: %s\n", args[i + 1].c_str());
         app.appState.TOPIC_CAMERA_NAME = args[i + 1];
      }
   }

   app.init(argc, argv);
   return app.exec();
}

