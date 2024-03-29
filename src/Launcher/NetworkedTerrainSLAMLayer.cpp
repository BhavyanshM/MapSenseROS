//
// Created by isr-lab on 1/10/22.
//

#include "NetworkedTerrainSLAMLayer.h"
#include "Scene/Mesh/MeshTools.h"

class MeshGenerator;

class PlanarRegion;

namespace Clay
{
   NetworkedTerrainSLAMLayer::NetworkedTerrainSLAMLayer(int argc, char **argv) : ApplicationLayer(argc, argv)
   {
      _data = new DataManager(appState, "/home/quantum/Workspace/Storage/Other/Temp/dataset/sequences/00/image_0/",
                              "/home/quantum/Workspace/Storage/Other/Temp/dataset/sequences/00/image_1/",
                              "/home/quantum/Workspace/Storage/Other/Temp/dataset/data_odometry_poses/poses/00.txt");

      /* KITTI:  float fx = 718.856, fy = 718.856, cx = 607.193, cy = 185.216; */
      /* L515 Color: fx = 602.25927734375, cx = 321.3750915527344, fy = 603.0400390625, cy = 240.51527404785156; */
//      _visualOdometry = new VisualOdometry(argc, argv, _networkManager, appState, _data);

      _regionCalculator = new PlanarRegionCalculator(argc, argv, appState);
      _regionCalculator->setOpenCLManager(_openCLManager);
      _mapper = new MapHandler(_networkManager, appState);
      _mapper->SetRegionCalculator(_regionCalculator);
      _mapper->SetSLAMModule(_slamModule);
      _mapper->SetMeshGenerator(&mesher);
      _slamModule = new SLAMModule(argc, argv);

      firstCloud = std::make_shared<PointCloud>(glm::vec4(0.7f, 0.4f, 0.5f, 1.0f), _rootModel);
      _models.emplace_back(std::dynamic_pointer_cast<Model>(firstCloud));

      std::string filename = ros::package::getPath("map_sense") + "/Extras/Clouds/Scan_" + std::to_string(90);
      CLAY_LOG_INFO("Loading Scan From: {}", filename);


      mesher.GeneratePoseMesh(Eigen::Matrix4f::Identity(), nullptr);

      /* ROS PointCloud*/
      PointCloudReceiver* pclReceiver = (PointCloudReceiver*) _networkManager->receivers[appState.OUSTER_POINTS];
      pclReceiver->SetRenderEnabled(false);
      _cloud = pclReceiver->GetRenderable();

      /* Static PointCloud from File. */
//      _cloud = std::make_shared<PointCloud>(glm::vec4(0.5f, 0.32f, 0.8f, 1.0f), _rootModel);
//      _cloud->Load(filename, false);

//       _models.emplace_back(std::dynamic_pointer_cast<Model>(_cloud));

       /* TODO: Do not delete! Refactor these image viewer lines into methods and classes.*/
//      _texture = Texture2D::Create(std::string(ASSETS_PATH) + std::string("Textures/Checkerboard.png"));
      _texture = Texture2D::Create();
//      _image = cv::imread("/home/quantum/Workspace/Storage/Other/Temp/dataset/sequences/00/image_0/003975.png");
      _image = cv::imread("/home/quantum/Pictures/OusterRegionComponents.png");

      cv::flip(_image, _image, 0);
      cv::flip(_image, _image, 1);

      _texture->LoadImage(_image.data, _image.cols, _image.rows, _image.channels());
   }

   void NetworkedTerrainSLAMLayer::MapsenseUpdate()
   {
      //      MS_DEBUG("TickEvent: %d", count++);

      //      if(_models.size() >=2) CLAY_LOG_INFO("Models: {} {} {}", _models.size(), _models[0]->GetSize(), _models[1]->GetSize());

      if (appState.ROS_ENABLED)
      {
         MS_DEBUG("ROS Update: {}", appState.ROS_ENABLED);
         _networkManager->SpinNode();
         _networkManager->AcceptMapsenseConfiguration(appState);
         _networkManager->ReceiverUpdate(appState);
         if (_networkManager->paramsAvailable)
         {
            _networkManager->paramsAvailable = false;
            appState.MERGE_DISTANCE_THRESHOLD = _networkManager->paramsMessage.mergeDistanceThreshold;
            appState.MERGE_ANGULAR_THRESHOLD = _networkManager->paramsMessage.mergeAngularThreshold;
         }

         if (appState.DEPTH_PLANAR_REGIONS_ENABLED)
         {
            cv::Mat depth;
            double inputTimestamp;
            ImageReceiver *depthReceiver = ((ImageReceiver *) this->_networkManager->receivers[appState.L515_DEPTH]);
            depthReceiver->getData(depth, appState, inputTimestamp);
            _regionCalculator->generateRegionsFromDepth(appState, depth, inputTimestamp);
            mesher.GenerateLineMeshForRegions(_regionCalculator->_depthRegionsZUp, nullptr, true);

            //
            //              // TODO: Fix this and publish planarregions msg
//            _networkManager->planarRegionPub.publish(_regionCalculator->publishRegions());
         }

         if(appState.HASH_PLANAR_REGIONS_ENABLED)
         {
            /* ROS Regions */
            if(_cloud->GetSize() > 0)_regionCalculator->GeneratePatchGraphFromPointCloud(appState, _cloud, 0.0);
            if(appState.USE_LINE_MESH)
            {
               mesher.ClearMeshes();
               mesher.GenerateLineMeshForRegions(_regionCalculator->planarRegionList, nullptr, true);
            }
            else
            {
               mesher.ClearLines();
               mesher.GenerateMeshForRegions(_regionCalculator->planarRegionList, nullptr, true);
            }

//            _networkManager->planarRegionPub.publish(_regionCalculator->publishRegions());
         }

         if (appState.STEREO_ODOMETRY_ENABLED)
         {
            MS_DEBUG("Stereo Odom Update");
            Clay::Ref<Clay::TriangleMesh> pose = std::make_shared<TriangleMesh>(glm::vec4(0.6f, 0.3f, 0.5f, 1.0f), _rootModel);
            MeshTools::CoordinateAxes(pose);
            _models.push_back(std::move(std::dynamic_pointer_cast<Model>(pose)));

            //            bool result = _visualOdometry->Update(appState, pose, firstCloud);
            //            _visualOdometry->Show();
         }

         if (appState.SLAM_ENABLED && _regionCalculator->planarRegionList.size() > 0 && _mapper->SLAM_ENABLED)
         {
            //            PlanarRegion::PrintRegionList(_regionCalculator->planarRegionList, "Initial Planar Regions");
            //            _slamModule->setLatestRegionsToZUp(_regionCalculator->planarRegionList);
            //            _slamModule->Update();

            /* TODO: Publish the latest optimized pose from Factor Graph SLAM. */

            //         vector<RigidBodyTransform> sensorTransforms = _slamModule->_mapper.poses;
            //         if (sensorTransforms.size() > 0 && _slamModule->enabled)
            //         {
            //            _networkManager->PublishPoseStamped(sensorTransforms.rbegin()[0]);
            //            printf("After SLAM Publisher.\n");
            //         }
         }

         if(_networkManager->GetSLAMPoses().size() > 0)
         {
            mesher.ClearPoses();
            for (auto pose : _networkManager->GetSLAMPoses())
            {
               _mapper->GetMesher()->GeneratePoseMesh(pose.second.GetMatrix().cast<float>(), nullptr);
            }
            _networkManager->ClearPoses();
         }

         MS_DEBUG("ROS Update Completed");
      }

      if (_regionCalculator->RenderEnabled())
      {
         _regionCalculator->Render();
      }

//      _networkManager->receivers[appState.OUSTER_POINTS]->render(appState);
   }

   void NetworkedTerrainSLAMLayer::ImGuiUpdate(ApplicationState& appState)
   {
      _networkManager->ImGuiUpdate(appState);
      _regionCalculator->ImGuiUpdate(appState);
      _slamModule->ImGuiUpdate();
      _mapper->ImGuiUpdate(appState);

      if(ImGui::BeginTabItem("Publisher"))
      {
         if(ImGui::Button("Publish Pose"))
         {
            RigidBodyTransform transform;
            _networkManager->PublishPoseStamped(transform, 0);
         }
         if(ImGui::Button("Publish Planes"))
         {
            _networkManager->PublishPlanes(_regionCalculator->planarRegionList, 0);
         }
         ImGui::EndTabItem();
      }

      /* TODO: Do not delete! Refactor these image viewer lines into methods and classes.*/
//      ImGui::Begin("Image");
//      uint32_t cbTextureId = _texture->GetRendererId();
//      ImVec2 region = ImGui::GetContentRegionAvail();
//      ImGui::Image((void *) cbTextureId, region, ImVec2{0, 1}, ImVec2(1, 0));
//
//      if (ImGui::IsItemHovered())
//      {
//         ImVec2 windowPos = ImGui::GetCursorScreenPos();
//         ImVec2 pos = ImGui::GetMousePos();
//         int x = (int) (pos.x - windowPos.x);
//         int y = (int)region.y + (int)(pos.y - windowPos.y);
//         ImGui::SetTooltip("N_X: %u\nN_Y: %u\nN_Z:%u", _image.at<cv::Vec3b>(y, x)[0], _image.at<cv::Vec3b>(y, x)[1], _image.at<cv::Vec3b>(y, x)[2]);
//      }
//      ImGui::End();

   }
}

