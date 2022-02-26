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

//   this->network = networkManager;
//
//   this->sensorPoseSub = new Subscriber();
//   *(this->sensorPoseSub) = networkManager->rosNode->subscribe("/atlas/sensors/chest_l515/pose", 3, &SLAMModule::sensorPoseCallback, this);

   this->extractArgs(argc, argv);

   _transformZUp.rotateZ(-90.0f / 180.0f * M_PI);
   _transformZUp.rotateY(90.0f / 180.0f * M_PI);

   // SLAM Unit Test Setup
   shared_ptr <PlanarRegion> region = std::make_shared<PlanarRegion>(0);
   region->SetToUnitSquare();

   RigidBodyTransform transform;
   transform.setRotationAndTranslation(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));

   region->transform(transform);
   _mapper->_testLatestRegions.emplace_back(region);

   _mapper->_atlasSensorPose.setRotationAndTranslation(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
}

void SLAMModule::extractArgs(int argc, char **argv)
{
   std::vector<std::string> args(argv, argv + argc);
   for (int i = 0; i < args.size(); i++)
   {
      if (args[i] == "--match-dist")
      {
         _mapper->MATCH_DIST_THRESHOLD = stof(args[i + 1]);
         ROS_INFO("DIST: %.3lf", _mapper->MATCH_DIST_THRESHOLD);
      }
      if (args[i] == "--match-dot")
      {
         _mapper->MATCH_ANGULAR_THRESHOLD = stof(args[i + 1]);
         ROS_INFO("ANGLE: %.3lf", _mapper->MATCH_ANGULAR_THRESHOLD);
      }
      if (args[i] == "--match-vert")
      {
         _mapper->MATCH_PERCENT_VERTEX_THRESHOLD = stoi(args[i + 1]);
         ROS_INFO("VERT: %d", _mapper->MATCH_PERCENT_VERTEX_THRESHOLD);
      }
      if (args[i] == "--factor-graph")
      {
         _mapper->FACTOR_GRAPH = true;
         ROS_INFO("Setting FACTOR_GRAPH: true");
      }
      if (args[i] == "--slam")
      {
         _mapper->SLAM_ENABLED = true;
         ROS_INFO("Setting SLAM_ENABLED: true");
      }
      if (args[i] == "--isam")
      {
         _mapper->ISAM2 = true;
         _mapper->ISAM2_NUM_STEPS = stoi(args[i + 1]);
         ROS_DEBUG("Setting ISAM2_NUM_STEPS: true \nSTEPS: %d\n", _mapper->ISAM2_NUM_STEPS);
      }
      if (args[i] == "--regions-dir")
      {
         std::string dirPath;
         dirPath = args[i + 1];
         _mapper->setDirectory(dirPath);
      }
   }
}

void SLAMModule::setLatestRegionsToZUp(const vector <shared_ptr<PlanarRegion>>& regions)
{
   _mapper->latestRegions = regions;
   _mapper->transformAndCopyRegions(_mapper->latestRegions, _mapper->_latestRegionsZUp, _transformZUp);
}

void SLAMModule::slamUpdate()
{
   if (!enabled)
      return;

   _mapper->MatchPlanarRegionsToMap();
   this->matchCountVec.emplace_back(_mapper->matches.size());
   ROS_DEBUG("Regions Matched: (%d).\n", _mapper->matches.size());

   if (ICPEnabled && _mapper->matches.size() > 0)
      _mapper->registerRegionsPointToPlane(1);

   int currentPoseId = 1;
   if (initial && poseAvailable)
   {
      initial = false;
      _mapper->_atlasSensorPose.print();
      _mapper->_atlasPreviousSensorPose.setMatrix(_mapper->_atlasSensorPose.getMatrix());
      _mapper->regions = _mapper->_latestRegionsZUp;
      _mapper->fgSLAM->addPriorPoseFactor(Eigen::MatrixXd(_mapper->_atlasSensorPose.getMatrix()));
      _mapper->fgSLAM->setPoseInitialValue(currentPoseId, Eigen::MatrixXd(_mapper->_atlasSensorPose.getMatrix()));
      poseAvailable = false;
   } else if (poseAvailable)
   {

      if (_mapper->FACTOR_GRAPH && (_mapper->_atlasSensorPose.getTranslation() - _mapper->_atlasPreviousSensorPose.getTranslation()).norm() > 0.001)
      {
         RigidBodyTransform odometry;
         odometry.setMatrix(_mapper->_atlasSensorPose.getMatrix());
         odometry.multiplyRight(_mapper->_atlasPreviousSensorPose.getInverse());

         //         printf("%d, %.4lf, %.4lf, %.4lf, %.4lf, %.4lf, %.4lf, %.4lf\n", _frameId, odometry.getTranslation().x(), odometry.getTranslation().y(),
         //                odometry.getTranslation().z(), odometry.getQuaternion().x(), odometry.getQuaternion().y(), odometry.getQuaternion().z(),
         //                odometry.getQuaternion().w());

         if (false)
         {
            GeomTools::saveRegions(_mapper->_latestRegionsZUp, ros::package::getPath("map_sense") + "/Extras/Regions/" +
                                                              string(4 - to_string(_frameId).length(), '0').append(to_string(_frameId)) + ".txt");
         }
         _frameId++;

         _mapper->fgSLAM->addPriorPoseFactor(Eigen::MatrixXd(_mapper->_atlasSensorPose.getMatrix()));

         currentPoseId = _mapper->fgSLAM->addOdometryFactor(Eigen::MatrixXd(odometry.getMatrix()));
         _mapper->fgSLAM->setPoseInitialValue(currentPoseId, Eigen::MatrixXd(_mapper->_atlasSensorPose.getMatrix()));
         _mapper->_atlasPreviousSensorPose.setMatrix(_mapper->_atlasSensorPose.getMatrix());

         /* Initialize poses and landmarks with map frame values. */
         _mapper->InsertOrientedPlaneFactors(currentPoseId);

         /* Transform and copy the latest planar regions from current sensor frame to map frame. NOTE: This copies ids from _latestRegionsZUp to regionsInMapFrame  */
         _mapper->transformAndCopyRegions(_mapper->_latestRegionsZUp, _mapper->regionsInMapFrame, _mapper->_atlasSensorPose);

         _mapper->SetOrientedPlaneInitialValues();
         _mapper->Optimize();

         renderSLAMOutput();

         if (_mapper->ISAM2)
            _mapper->fgSLAM->clearISAM2();
      } else
      {
         //         _mapper->poses.emplace_back(RigidBodyTransform(_mapper->_sensorToMapTransform));
      }

      ROS_DEBUG("Recycling Region Lists.\n");
      /* Load previous and current regions. Separated by SKIP_REGIONS. */
      _mapper->regions = _mapper->_latestRegionsZUp;
      poseAvailable = false;

      ROS_DEBUG("SLAM Update Complete.\n");
   }
}

void SLAMModule::renderSLAMOutput()
{
   /* Generate World Frame Region Mesh. */
   //         for (int k = 0; k < _mapper->regionsInMapFrame.size(); k++)
   //            _mapper->regionsInMapFrame[k]->RetainConvexHull();

   /* Extracting Landmarks from Factor Graph. */
   _mapper->ExtractFactorGraphLandmarks();

   /* Reducing Vertex Count on Region Boundaries. */
   for (int k = 0; k < _mapper->mapRegions.size(); k++)
      _mapper->mapRegions[k]->RetainConvexHull();

   /* Generating Pose Mesh */
   _mapper->fgSLAM->getPoses(_mapper->poses);

   _mapper->atlasPoses.emplace_back(_mapper->_atlasSensorPose);
}

void SLAMModule::SLAMTesterUpdate()
{
   //   AppUtils::getFileNames(_mapper->directory, _mapper->files, true);
   //   GeomTools::loadRegions(_frameId, fileRegions, _mapper->directory, _mapper->files);

   //   Eigen::Vector3d position;
   //   Quaterniond orientation;
   //
   //   cout << _mapper->directory + "poses.txt" << endl;
   //   ifstream fs(_mapper->directory + "poses.txt");
   //   GeomTools::loadPoseStamped(fs, position, orientation);

   //   _frameId++;

   /* Unit Test Visualization. */


   cout << "File SLAM End" << endl;

   slamUpdate();
}

void SLAMModule::ImGuiUpdate()
{
   if (ImGui::BeginTabItem("SLAM"))
   {
      ImGui::Text("SLAM");
      ImGui::EndTabItem();


      ImPlot::ShowDemoWindow(&render);

      //   if (this->matchCountVec.size() > 20)
      //   {
      //      this->matchCountVec.erase(this->matchCountVec.begin());
      //      int *data = this->matchCountVec.data();
      //      if (ImPlot::BeginPlot("SLAM Plots"))
      //      {
      //         ImPlot::PlotLine("Line Plot", data, 10);
      //         ImPlot::EndPlot();
      //      }
      //   }
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
