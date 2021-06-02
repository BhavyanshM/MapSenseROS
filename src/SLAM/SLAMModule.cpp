//
// Created by quantum on 5/28/21.
//

#include "SLAMModule.h"

SLAMModule::SLAMModule(int argc, char **argv, NetworkManager *networkManager)
{
   this->network = networkManager;

   this->sensorPoseSub = new Subscriber();
   *(this->sensorPoseSub) = networkManager->rosNode->subscribe("/atlas/sensors/chest_l515/pose", 3, &SLAMModule::sensorPoseCallback, this);

   this->extractArgs(argc, argv);
   this->init();
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

void SLAMModule::init()
{
   printf("Initializing SLAM Module\n");
   _mapper.fgSLAM->addPriorPoseFactor(Pose3().identity(), 1);
   _mapper.fgSLAM->initPoseValue(Pose3().identity());
   printf("SLAM Module Initialized \n");
}

void SLAMModule::slamUpdate(vector<shared_ptr<PlanarRegion>> latestRegions)
{
   if (!enabled)
      return;

   printf("SLAM Update Begins\n");

   if (_mapper.regions.size() == 0)
   {
      printf("Setting Regions to Latest: %d\n", latestRegions.size());
      _mapper.regions = latestRegions;
      printf("Regions Copied into SLAMModule.\n");
      return;
   }

   auto start = high_resolution_clock::now();

   _mapper.latestRegions = latestRegions;

   /* Match the previous and latest regions to copy ids and generate match indices. Calculate odometry between previous and latest poses. */
   _mapper.matchPlanarRegionsToMap(_mapper.latestRegions);

   this->matchCountVec.emplace_back(_mapper.matches.size());

   printf("SLAM Update: (Region Matches Found: %d)\n", _mapper.matches.size());

   if (_mapper.matches.size() > 0)
      _mapper.registerRegionsPointToPlane(1);

   printf("SLAM Update: (Regions Registered)\n");

   /* Insert the local landmark measurements and odometry constraints into the Factor Graph for SLAM. */
   int currentPoseId = _mapper.updateFactorGraphPoses(_mapper._sensorPoseRelative);
   _mapper.updateFactorGraphLandmarks(_mapper.latestRegions, currentPoseId);

   printf("SLAM Update: (Factors Inserted)\n");

   /* Transform and copy the latest planar regions from current sensor frame to map frame.  */
   vector<shared_ptr<PlanarRegion>> regionsInMapFrame;
   _mapper.transformAndCopyLatestRegions(_mapper._sensorToMapTransform, regionsInMapFrame);

   if (_mapper.FACTOR_GRAPH)
   {
      /* Initialize poses and landmarks with map frame values. */
      _mapper.initFactorGraphState(_mapper._sensorToMapTransform.getInverse(), regionsInMapFrame);
      //            _mapper.fgSLAM->addPriorPoseFactor(Pose3(_mapper._sensorToMapTransform.getInverse().getMatrix()), currentPoseId);

      /* Optimize the Factor Graph and get Results. */
      _mapper.ISAM2 ? _mapper.fgSLAM->optimizeISAM2(_mapper.ISAM2_NUM_STEPS) : _mapper.fgSLAM->optimize();
      //_mapper.mergeLatestRegions();
      _mapper.updateMapRegionsWithSLAM();

      auto end = high_resolution_clock::now();
      auto duration = duration_cast<microseconds>(end - start).count();
      printf("SLAM Update Took: %.2lf ms\n", duration / (float) 1000);

      //      for (shared_ptr<PlanarRegion> region : _mapper.mapRegions)
      //         GeomTools::compressPointSetLinear(region);

      //      _mesher.generateRegionLineMesh(_mapper.mapRegions, latestRegionEdges, frameIndex, _sensor);
      _mapper.fgSLAM->getPoses(_mapper.poses);

      if (_mapper.ISAM2)
         _mapper.fgSLAM->clearISAM2();
   } else
   {
      _mapper.poses.emplace_back(RigidBodyTransform(_mapper._sensorToMapTransform));
      //      _mesher.generateRegionLineMesh(regionsInMapFrame, latestRegionEdges, frameIndex, _sensor);
   }

   printf("SLAM Update: (Factor Graph Optimized)\n");

   //   _mesher.generatePoseMesh(_mapper.poses, poseAxes, _sensor);

   /* Load previous and current regions. Separated by SKIP_REGIONS. */
   _mapper.regions = _mapper.latestRegions;

   //   _mesher.generateMatchLineMesh(_mapper.matches, _mapper.regions, _mapper.latestRegions, matchingEdges, _sensor);

   printf("SLAM Update Ends\n");
}

void SLAMModule::ImGuiUpdate()
{
   ImGui::Checkbox("Enabled", &this->enabled);
   if (this->matchCountVec.size() > 20)
   {
      printf("Plotting ImGui (%d)\n", this->matchCountVec.size());
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
   this->sensorPoseMessage = pose;
   this->_mapper._atlasSensorPose.setRotationAndTranslation(Eigen::Quaterniond(this->sensorPoseMessage->pose.orientation.x,
                                                                               this->sensorPoseMessage->pose.orientation.y,
                                                                               this->sensorPoseMessage->pose.orientation.z,
                                                                               this->sensorPoseMessage->pose.orientation.w),
                                                            Eigen::Vector3d(this->sensorPoseMessage->pose.position.x,
                                                                            this->sensorPoseMessage->pose.position.y,
                                                                            this->sensorPoseMessage->pose.position.z));
   ROS_INFO("Sensor Pose Received.");
   this->_mapper._atlasSensorPose.print();
};