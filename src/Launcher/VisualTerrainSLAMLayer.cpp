//
// Created by isr-lab on 1/10/22.
//

#include "VisualTerrainSLAMLayer.h"
#include "Scene/Mesh/MeshTools.h"

namespace Clay
{

   VisualTerrainSLAMLayer::VisualTerrainSLAMLayer(int argc, char **argv) : ApplicationLauncher(argc, argv)
   {
//      _kitti = new DataManager(appState, "/home/quantum/Workspace/Storage/Other/Temp/dataset/sequences/00/image_0/",
//                               "/home/quantum/Workspace/Storage/Other/Temp/dataset/sequences/00/image_1/",
//                               "/home/quantum/Workspace/Storage/Other/Temp/dataset/data_odometry_poses/poses/00.txt");
//
//      /* KITTI:  float fx = 718.856, fy = 718.856, cx = 607.193, cy = 185.216; */
//      ROS_INFO("VisualOdometry Created.");
//      _kitti->SetCamera(CameraParams(718.856, 718.856, 607.193, 185.216), CameraParams(718.856, 718.856, 607.193, 185.216));
//
//      _visualOdometry = new VisualOdometry(argc, argv, _networkManager, appState, _kitti);


      _regionCalculator = new PlanarRegionCalculator(argc, argv, appState);
      _regionCalculator->setOpenCLManager(_openCLManager);

      firstCloud = std::make_shared<PointCloud>(glm::vec4(0.7f, 0.4f, 0.5f, 1.0f), _rootPCL);
      //      _visualOdometry->Initialize(firstCloud);
      _models.emplace_back(std::dynamic_pointer_cast<Model>(firstCloud));
   }

   void VisualTerrainSLAMLayer::MapsenseUpdate()
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
            Clay::Ref<Clay::TriangleMesh> pose = std::make_shared<TriangleMesh>(glm::vec4(0.6f, 0.3f, 0.5f, 1.0f), _rootPCL);
            MeshTools::CoordinateAxes(pose);
            _poses.push_back(std::move(std::dynamic_pointer_cast<Model>(pose)));

            _visualOdometry->Update(appState, pose, firstCloud);
            _visualOdometry->Show();
         }

         if (appState.SLAM_ENABLED && _regionCalculator->planarRegionList.size() > 0 && _slamModule->_mapper.SLAM_ENABLED)
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
   }

   void VisualTerrainSLAMLayer::ImGuiUpdate(ApplicationState& appState)
   {
      _networkManager->ImGuiUpdate(appState);
      _regionCalculator->ImGuiUpdate(appState);
   }
}
