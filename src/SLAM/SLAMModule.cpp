//
// Created by quantum on 5/28/21.
//

#include "SLAMModule.h"
#include "imgui.h"
#include "implot.h"

SLAMModule::SLAMModule(int argc, char **argv)
{

   /* TODO: Fix this mess before any SLAM ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

//   this->extractArgs(argc, argv);



}

//void SLAMModule::extractArgs(int argc, char **argv)
//{
//   std::vector<std::string> args(argv, argv + argc);
//   for (int i = 0; i < args.size(); i++)
//   {
//      if (args[i] == "--match-dist")
//      {
//         _mapper->MATCH_DIST_THRESHOLD = stof(args[i + 1]);
//         ROS_INFO("DIST: %.3lf", _mapper->MATCH_DIST_THRESHOLD);
//      }
//      if (args[i] == "--match-dot")
//      {
//         _mapper->MATCH_ANGULAR_THRESHOLD = stof(args[i + 1]);
//         ROS_INFO("ANGLE: %.3lf", _mapper->MATCH_ANGULAR_THRESHOLD);
//      }
//      if (args[i] == "--match-vert")
//      {
//         _mapper->MATCH_PERCENT_VERTEX_THRESHOLD = stoi(args[i + 1]);
//         ROS_INFO("VERT: %d", _mapper->MATCH_PERCENT_VERTEX_THRESHOLD);
//      }
//      if (args[i] == "--factor-graph")
//      {
//         _mapper->FACTOR_GRAPH = true;
//         ROS_INFO("Setting FACTOR_GRAPH: true");
//      }
//      if (args[i] == "--slam")
//      {
//         _mapper->SLAM_ENABLED = true;
//         ROS_INFO("Setting SLAM_ENABLED: true");
//      }
//      if (args[i] == "--isam")
//      {
//         _mapper->ISAM2 = true;
//         _mapper->ISAM2_NUM_STEPS = stoi(args[i + 1]);
//         ROS_DEBUG("Setting ISAM2_NUM_STEPS: true \nSTEPS: %d\n", _mapper->ISAM2_NUM_STEPS);
//      }
//      if (args[i] == "--regions-dir")
//      {
//         std::string dirPath;
//         dirPath = args[i + 1];
//         _mapper->setDirectory(dirPath);
//      }
//   }
//}



void SLAMModule::renderSLAMOutput()
{
//   /* Generate World Frame Region Mesh. */
//   //         for (int k = 0; k < _mapper->regionsInMapFrame.size(); k++)
//   //            _mapper->regionsInMapFrame[k]->RetainConvexHull();
//
//   /* Extracting Landmarks from Factor Graph. */
//   _mapper->ExtractFactorGraphLandmarks();
//
//   /* Reducing Vertex Count on Region Boundaries. */
//   for (int k = 0; k < _mapper->_mapRegions.size(); k++)
//      _mapper->_mapRegions[k]->RetainConvexHull();
//
//   /* Generating Pose Mesh */
//   _mapper->fgSLAM->getPoses(_mapper->poses);
//
//   _mapper->atlasPoses.emplace_back(_mapper->_atlasSensorPose);
}

void SLAMModule::ImGuiUpdate()
{
   if (ImGui::BeginTabItem("SLAM"))
   {
      ImGui::Text("SLAM");
      ImGui::EndTabItem();


      ImPlot::ShowDemoWindow(&render);

   }
}



//void SLAMModule::sensorPoseCallback(const geometry_msgs::PoseStampedConstPtr& poseMsg)
//{
//   if (_sensorPoseMessage == nullptr)
//   {
//      _sensorPoseMessage = poseMsg;
//      return;
//   }
//
//   if (!poseAvailable)
//   {
//
//      Eigen::Vector3d position;
//      Eigen::Vector3d previous;
//
//      position << poseMsg->pose.position.x, poseMsg->pose.position.y, poseMsg->pose.position.z;
//      previous << this->_sensorPoseMessage->pose.position.x, this->_sensorPoseMessage->pose.position.y, this->_sensorPoseMessage->pose.position.z;
//
//      this->_sensorPoseMessage = poseMsg;
//
//      double diff = (position - previous).norm();
//
//      //      if(diff > 0.001)
//      {
//         ROS_DEBUG("Distance From Last Transform: (%.4lf)\n", diff);
//
//         this->_mapper->_atlasSensorPose.setRotationAndTranslation(
//               Eigen::Quaterniond(this->_sensorPoseMessage->pose.orientation.w, this->_sensorPoseMessage->pose.orientation.x,
//                                  this->_sensorPoseMessage->pose.orientation.y, this->_sensorPoseMessage->pose.orientation.z),
//               Eigen::Eigen::Vector3d(this->_sensorPoseMessage->pose.position.x, this->_sensorPoseMessage->pose.position.y,
//                               this->_sensorPoseMessage->pose.position.z));
//
//         poseAvailable = true;
//      }
//   }
//}
