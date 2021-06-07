//
// Created by quantum on 5/28/21.
//

#include "SLAMModule.h"

SLAMModule::SLAMModule(int argc, char **argv, NetworkManager *networkManager, Magnum::SceneGraph::DrawableGroup3D *_drawables, Object3D *sensor) : _mesher(
      _drawables), _sensor(sensor)
{
   this->network = networkManager;

   this->sensorPoseSub = new Subscriber();
   *(this->sensorPoseSub) = networkManager->rosNode->subscribe("/atlas/sensors/chest_l515/pose", 3, &SLAMModule::sensorPoseCallback, this);

   this->extractArgs(argc, argv);
}

void SLAMModule::extractArgs(int argc, char **argv)
{
   std::vector<std::string> args(argv, argv + argc);
   for (int i = 0; i < args.size(); i++)
   {
      if (args[i] == "--match-dist")
      {
         _mapper.MATCH_DIST_THRESHOLD = stof(args[i + 1]);
         cout << "DIST:" << _mapper.MATCH_DIST_THRESHOLD << "\t";
      }
      if (args[i] == "--match-dot")
      {
         _mapper.MATCH_ANGULAR_THRESHOLD = stof(args[i + 1]);
         cout << "ANGLE:" << _mapper.MATCH_ANGULAR_THRESHOLD << "\t";
      }
      if (args[i] == "--match-vert")
      {
         _mapper.MATCH_PERCENT_VERTEX_THRESHOLD = stoi(args[i + 1]);
         cout << "VERT:" << _mapper.MATCH_PERCENT_VERTEX_THRESHOLD << "\n";
      }
      if (args[i] == "--factor-graph")
      {
         _mapper.FACTOR_GRAPH = true;
         cout << "Factor Graph: true" << endl;
      }
      if (args[i] == "--isam")
      {
         _mapper.ISAM2 = true;
         _mapper.ISAM2_NUM_STEPS = stoi(args[i + 1]);
         printf("Incremental SAM2: true \nSTEPS: %d\n", _mapper.ISAM2_NUM_STEPS);
      }
   }
}

void SLAMModule::slamUpdate(const vector<shared_ptr<PlanarRegion>>& latestRegions)
{
   if (!enabled)
      return;

   //   if (_mapper.regions.size() == 0)
   //   {
   //      _mapper.regions = latestRegions;
   //      return;
   //   }


   _mapper.latestRegions = latestRegions;

   /* Match the previous and latest regions to copy ids and generate match indices. Calculate odometry between previous and latest poses. */
   _mapper.matchPlanarRegionsToMap();
   this->matchCountVec.emplace_back(_mapper.matches.size());

   printf("Regions Matched: (%d).\n", _mapper.matches.size());

   if (ICPEnabled && _mapper.matches.size() > 0)
      _mapper.registerRegionsPointToPlane(1);

   int currentPoseId = 1;
   /* Insert the local landmark measurements and odometry constraints into the Factor Graph for SLAM. */
   if (initial && poseAvailable)
   {
      printf("Initialization\n");
      initial = false;
      _mapper._atlasSensorPose.print();

      printf("Initialized\n");

      _mapper._atlasPreviousSensorPose.setMatrix(_mapper._atlasSensorPose.getMatrix());
      _mapper.regions = _mapper.latestRegions;

      poseAvailable = false;
      _mapper.fgSLAM->addPriorPoseFactor(MatrixXd(_mapper._atlasSensorPose.getMatrix()));

      _mapper.fgSLAM->setPoseInitialValue(currentPoseId, MatrixXd(_mapper._atlasSensorPose.getMatrix()));
   } else if (poseAvailable)
   {

      /* Transform and copy the latest planar regions from current sensor frame to map frame.  */
      _mapper.transformAndCopyLatestRegions(_mapper.regionsInMapFrame, _mapper._atlasSensorPose);

      if (_mapper.FACTOR_GRAPH)
      {

         RigidBodyTransform odometry(_mapper._atlasSensorPose);
         odometry.multiplyLeft(_mapper._atlasPreviousSensorPose.getInverse());

         currentPoseId = _mapper.fgSLAM->addOdometryFactor(MatrixXd(odometry.getMatrix()));
         _mapper.fgSLAM->setPoseInitialValue(currentPoseId, MatrixXd(_mapper._atlasSensorPose.getMatrix()));
         _mapper._atlasPreviousSensorPose.setMatrix(_mapper._atlasSensorPose.getMatrix());

         /* Initialize poses and landmarks with map frame values. */
         //         _mapper.insertOrientedPlaneFactors(currentPoseId);
         //         _mapper.setOrientedPlaneInitialValues();

         //         _mapper.printRefCounts();

         /* Optimize the Factor Graph and get Results. */
         _mapper.optimize();
         //_mapper.mergeLatestRegions();

         _mesher.generateRegionLineMesh(_mapper.regionsInMapFrame, latestRegionEdges, 1, _sensor, false);

         //         printf("Extracting Factor Graph Landmarks\n");
         //         _mapper.extractFactorGraphLandmarks();
         //
         //         printf("Generating Region Mesh\n");
         //         _mesher.generateRegionLineMesh(_mapper.mapRegions, latestRegionEdges, _mapper.fgSLAM->getPoseId(), _sensor);


         _mapper.fgSLAM->getPoses(_mapper.poses);

         printf("Clearing ISAM2.\n");

         if (_mapper.ISAM2)
            _mapper.fgSLAM->clearISAM2();
      } else
      {
         _mapper.poses.emplace_back(RigidBodyTransform(_mapper._sensorToMapTransform));
         //      _mesher.generateRegionLineMesh(_mapper.regionsInMapFrame, latestRegionEdges, frameIndex, _sensor);
      }

      printf("Appending Pose Mesh\n");
      //      _mesher.appendPoseMesh(_mapper._atlasSensorPose, atlasPoseAxes, _sensor, 1);
      _mesher.generatePoseMesh(_mapper.poses, poseAxes, _sensor, 2);

      printf("Recycling Region Lists.\n");
      /* Load previous and current regions. Separated by SKIP_REGIONS. */
      _mapper.regions = _mapper.latestRegions;
      poseAvailable = false;

      printf("SLAM Update Complete.\n");

      //      _mapper.printRefCounts();
   }
}

void SLAMModule::ImGuiUpdate()
{
   ImGui::Checkbox("SLAM Enabled", &this->enabled);
   ImGui::Checkbox("ICP Enabled", &this->ICPEnabled);
   if (this->matchCountVec.size() > 20)
   {

      this->matchCountVec.erase(this->matchCountVec.begin());
      int *data = this->matchCountVec.data();
      if (ImPlot::BeginPlot("SLAM Plots"))
      {
         ImPlot::PlotLine("Line Plot", data, 10);
         ImPlot::EndPlot();
      }
   }
}

void SLAMModule::sensorPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose)
{
   if (!poseAvailable)
   {
      this->sensorPoseMessage = pose;
      this->_mapper._atlasSensorPose.setRotationAndTranslation(
            Eigen::Quaterniond(this->sensorPoseMessage->pose.orientation.x, this->sensorPoseMessage->pose.orientation.y,
                               this->sensorPoseMessage->pose.orientation.z, this->sensorPoseMessage->pose.orientation.w),
            Eigen::Vector3d(this->sensorPoseMessage->pose.position.x, this->sensorPoseMessage->pose.position.y, this->sensorPoseMessage->pose.position.z));

      poseAvailable = true;
      ROS_INFO("Sensor Pose Received.");
      //   this->_mapper._atlasSensorPose.print();
   };
}
