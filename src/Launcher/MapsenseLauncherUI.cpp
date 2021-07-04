#include "MapsenseLauncherUI.h"

MyApplication::MyApplication(const Arguments& arguments) : Magnum::Platform::Application{arguments,
                                                                                         Configuration{}.setTitle("MapsenseLauncherUI").setSize(
                                                                                               Magnum::Vector2i(1920, 1080)).setWindowFlags(
                                                                                               Configuration::WindowFlag::Resizable)}
{

   _imgui = Magnum::ImGuiIntegration::Context(Magnum::Vector2{windowSize()} / dpiScaling(), windowSize(), framebufferSize());
   ImPlot::CreateContext();

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
   _camParent = new Object3D{_camGrandParent};
   _camObject = new Object3D{_camParent};
   _camera = new Magnum::SceneGraph::Camera3D(*_camObject);
   _camera->setProjectionMatrix(Magnum::Matrix4::perspectiveProjection(35.0_degf, 1.33f, 0.001f, 100.0f));

   _world = new Object3D{&_scene};
   _world->transformLocal(Magnum::Matrix4::rotationX(Magnum::Rad{-90.0_degf}));
   _world->transformLocal(Magnum::Matrix4::rotationZ(Magnum::Rad{90.0_degf}));

   /* TODO: Prepare your objects here and add them to the scene */
   _sensorAxes = new Object3D{_world};
   _sensorAxes->scale({0.1, 0.1, 0.1});
   new DrawableObject{*_sensorAxes, &_drawables, Magnum::Primitives::axis3D(), {0.5, 0.3f, 0.4f}};

   _camObject->translate({0, 0, 10.0f});

   _camOriginCube = new Object3D{_camGrandParent};
   _camOriginCube->scale({0.01f, 0.01f, 0.01f});
   new DrawableObject{*_camOriginCube, &_drawables, Magnum::Primitives::cubeSolid(), {0.2, 0.0f, 0.3f}};

   _mesher = new MeshGenerator(&_drawables);

   // setMinimalLoopPeriod(32); /* Needs to be less than 30-32 milliseconds for real-time performance */
}

void MyApplication::init(int argc, char **argv)
{
   Log::Init();

   _openCLManager = new OpenCLManager();

   /* TODO: Instantiate the Information Processors */
   _networkManager = new NetworkManager(appState, &appUtils);
   _networkManager->init_ros_node(argc, argv, appState);

   _regionCalculator = new PlanarRegionCalculator(argc, argv, _networkManager, appState);
   _regionCalculator->setOpenCLManager(_openCLManager);

   _keypointDetector = new KeypointDetector(argc, argv, _networkManager, appState);

   _slamModule = new SLAMModule(argc, argv, _networkManager, &_drawables, _world);

   ROS_DEBUG("Application Initialized Successfully");
}

void MyApplication::tickEvent()
{
   ROS_DEBUG("TickEvent: %d", count++);

   if (appState.ROS_ENABLED)
   {
      _networkManager->spin_ros_node();
      _networkManager->acceptMapsenseConfiguration(appState);
      _networkManager->receiverUpdate(appState);
      if (_networkManager->paramsAvailable)
      {
         _networkManager->paramsAvailable = false;
         appState.MERGE_DISTANCE_THRESHOLD = _networkManager->paramsMessage.mergeDistanceThreshold;
         appState.MERGE_ANGULAR_THRESHOLD = _networkManager->paramsMessage.mergeAngularThreshold;
      }

      auto start = std::chrono::high_resolution_clock::now();
      _regionCalculator->generateAndPublishRegions(appState);
      auto end = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

      averageTime += duration;
      timeCount += 1;

      MAPSENSE_LOG_INFO("Found Rings in {0}\tms", (float) averageTime / ((float) timeCount * 1000));

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

      if (appState.SHOW_REGION_EDGES)
      {
         MeshGenerator::clearMesh(regionEdges);
         _mesher->generateRegionLineMesh(_regionCalculator->planarRegionList, regionEdges, 4, _world, true);
      }
   }

   /* Update Without ROS */
   else
   {
      if (count % 10 == 0)
      {
         ROS_DEBUG("Test SLAM Update.");
         _slamModule->SLAMTesterUpdate();
      }
   }
}



