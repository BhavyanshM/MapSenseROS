//
// Created by isr-lab on 1/10/22.
//

#include "NetworkedTerrainSLAMLayer.h"
#include "Scene/Mesh/MeshTools.h"
#include "MeshGenerator.h"
#include "PlanarRegion.h"

namespace Clay
{
   NetworkedTerrainSLAMLayer::NetworkedTerrainSLAMLayer(int argc, char **argv) : ApplicationLayer(argc, argv)
   {
      _data = new DataManager(appState, "/home/quantum/Workspace/Storage/Other/Temp/dataset/sequences/00/image_0/",
                               "/home/quantum/Workspace/Storage/Other/Temp/dataset/sequences/00/image_1/",
                               "/home/quantum/Workspace/Storage/Other/Temp/dataset/data_odometry_poses/poses/00.txt");

      /* KITTI:  float fx = 718.856, fy = 718.856, cx = 607.193, cy = 185.216; */
      /* L515 Color: fx = 602.25927734375, cx = 321.3750915527344, fy = 603.0400390625, cy = 240.51527404785156; */
      _visualOdometry = new VisualOdometry(argc, argv, _networkManager, appState, _data);

      _regionCalculator = new PlanarRegionCalculator(argc, argv, appState);
      _regionCalculator->setOpenCLManager(_openCLManager);
      _mapper = new PlanarRegionMapHandler();
      _mapper->SetRegionCalculator(_regionCalculator);

      firstCloud = std::make_shared<PointCloud>(glm::vec4(0.7f, 0.4f, 0.5f, 1.0f), _rootModel);
      _models.emplace_back(std::dynamic_pointer_cast<Model>(firstCloud));

      std::string filename = ros::package::getPath("map_sense") + "/Extras/Clouds/Scan_" + std::to_string(90);
      CLAY_LOG_INFO("Loading Scan From: {}", filename);

      PointCloudReceiver* pclReceiver = (PointCloudReceiver*) _networkManager->receivers[appState.OUSTER_POINTS];
      _cloud = pclReceiver->GetRenderable();
      _models.emplace_back(std::dynamic_pointer_cast<Clay::Model>(_cloud));

//      cloud = std::make_shared<PointCloud>(glm::vec4(0.5f, 0.32f, 0.8f, 1.0f), _rootModel);
//      cloud->Load(filename, false);
//      _models.emplace_back(std::dynamic_pointer_cast<Model>(cloud));

//      _regionCalculator->GeneratePatchGraphFromPointCloud(appState, cloud->GetMesh()->_vertices, 0.0);

   }

   void NetworkedTerrainSLAMLayer::MapsenseUpdate()
   {
      //      ROS_DEBUG("TickEvent: %d", count++);

      if(_models.size() >=2) CLAY_LOG_INFO("Models: {} {} {}", _models.size(), _models[0]->GetSize(), _models[1]->GetSize());

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

         if (appState.PLANAR_REGIONS_ENABLED)
         {
            cv::Mat depth;
            double inputTimestamp;
            ImageReceiver *depthReceiver = ((ImageReceiver *) this->_networkManager->receivers[appState.L515_DEPTH]);
            depthReceiver->getData(depth, appState, inputTimestamp);
            _regionCalculator->generateRegionsFromDepth(appState, depth, inputTimestamp);
            _regionCalculator->Render();
            //
            //              // TODO: Fix this and publish planarregions msg
            //              _networkManager->planarRegionPub.publish(_regionCalculator->publishRegions());
         }

         if (appState.STEREO_ODOMETRY_ENABLED)
         {
            ROS_DEBUG("Stereo Odom Update");
            Clay::Ref<Clay::TriangleMesh> pose = std::make_shared<TriangleMesh>(glm::vec4(0.6f, 0.3f, 0.5f, 1.0f), _rootModel);
            MeshTools::CoordinateAxes(pose);
            _poses.push_back(std::move(std::dynamic_pointer_cast<Model>(pose)));

            bool result = _visualOdometry->Update(appState, pose, firstCloud);
             _visualOdometry->Show();
         }

         if (appState.SLAM_ENABLED && _regionCalculator->planarRegionList.size() > 0 && _mapper->SLAM_ENABLED)
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

         ROS_DEBUG("ROS Update Completed");
      }

      if(_regionCalculator->RenderEnabled())
      {
//         _cloud = ((PointCloudReceiver*)_networkManager->receivers[appState.OUSTER_POINTS])->GetNextCloud();
         if(_cloud->GetSize() > 0)_regionCalculator->GeneratePatchGraphFromPointCloud(appState, _cloud->GetMesh()->_vertices, 0.0);
         mesher.GenerateMeshForRegions(_regionCalculator->planarRegionList, nullptr);
         _regionCalculator->Render();
      }

      _networkManager->receivers[appState.OUSTER_POINTS]->render(appState);


   }

   void NetworkedTerrainSLAMLayer::ImGuiUpdate(ApplicationState& appState)
   {
      _networkManager->ImGuiUpdate(appState);
      _regionCalculator->ImGuiUpdate(appState);
      _slamModule->ImGuiUpdate();
      _mapper->ImGuiUpdate();

   }
}

