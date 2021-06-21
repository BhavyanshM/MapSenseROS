//
// Created by quantum on 5/28/21.
//

#include "SLAMModule.h"

SLAMModule::SLAMModule(int argc, char **argv, NetworkManager *networkManager, Magnum::SceneGraph::DrawableGroup3D *_drawables, Object3D *sensor) : _mesher(
      _drawables), _world(sensor)
{
   this->network = networkManager;

   this->sensorPoseSub = new Subscriber();
   *(this->sensorPoseSub) = networkManager->rosNode->subscribe("/atlas/sensors/chest_l515/pose", 3, &SLAMModule::sensorPoseCallback, this);

   this->extractArgs(argc, argv);

   _transformZUp.rotateZ(-90.0f / 180.0f * M_PI);
   _transformZUp.rotateY(90.0f / 180.0f * M_PI);
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
         ROS_DEBUG("Incremental SAM2: true \nSTEPS: %d\n", _mapper.ISAM2_NUM_STEPS);
      }
      if (args[i] == "--regions-dir")
      {
         std::string dirPath;
         dirPath = args[i + 1];
         _mapper.setDirectory(dirPath);
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
   _mapper.transformAndCopyRegions(_mapper.latestRegions, _mapper._latestRegionsZUp, _transformZUp);

   /* Match the previous and latest regions to copy ids and generate match indices. Calculate odometry between previous and latest poses. */
   _mapper.matchPlanarRegionsToMap();
   this->matchCountVec.emplace_back(_mapper.matches.size());

   ROS_DEBUG("Regions Matched: (%d).\n", _mapper.matches.size());

   if (ICPEnabled && _mapper.matches.size() > 0)
      _mapper.registerRegionsPointToPlane(1);

   int currentPoseId = 1;
   /* Insert the local landmark measurements and odometry constraints into the Factor Graph for SLAM. */
   if (initial && poseAvailable)
   {
      initial = false;
      _mapper._atlasSensorPose.print();

      _mapper._atlasPreviousSensorPose.setMatrix(_mapper._atlasSensorPose.getMatrix());
      _mapper.regions = _mapper._latestRegionsZUp;

      _mapper.fgSLAM->addPriorPoseFactor(MatrixXd(_mapper._atlasSensorPose.getMatrix()));
      _mapper.fgSLAM->setPoseInitialValue(currentPoseId, MatrixXd(_mapper._atlasSensorPose.getMatrix()));

      poseAvailable = false;

   } else if (poseAvailable)
   {

      /* Transform and copy the latest planar regions from current sensor frame to map frame.  */
      _mapper.transformAndCopyRegions(_mapper._latestRegionsZUp, _mapper.regionsInMapFrame, _mapper._atlasSensorPose);

      if (_mapper.FACTOR_GRAPH && (_mapper._atlasSensorPose.getTranslation() - _mapper._atlasPreviousSensorPose.getTranslation()).norm() > 0.001)
      {

         RigidBodyTransform odometry;
         odometry.setMatrix(_mapper._atlasSensorPose.getMatrix());
         odometry.multiplyRight(_mapper._atlasPreviousSensorPose.getInverse());

         printf("%d, %.4lf, %.4lf, %.4lf, %.4lf, %.4lf, %.4lf, %.4lf\n", _frameId, odometry.getTranslation().x(), odometry.getTranslation().y(), odometry.getTranslation().z(),
                  odometry.getQuaternion().x(), odometry.getQuaternion().y(), odometry.getQuaternion().z(), odometry.getQuaternion().w());

         if(false)
         {
            GeomTools::saveRegions(_mapper._latestRegionsZUp, ros::package::getPath("map_sense") + "/Extras/Regions/" +
                                                          string(4 - to_string(_frameId).length(), '0').append(to_string(_frameId)) + ".txt");
         }
         _frameId++;

//         _mapper.fgSLAM->addPriorPoseFactor(MatrixXd(_mapper._atlasSensorPose.getMatrix()));

         currentPoseId = _mapper.fgSLAM->addOdometryFactor(MatrixXd(odometry.getMatrix()));
         _mapper.fgSLAM->setPoseInitialValue(currentPoseId, MatrixXd(_mapper._atlasSensorPose.getMatrix()));
         _mapper._atlasPreviousSensorPose.setMatrix(_mapper._atlasSensorPose.getMatrix());

         /* Initialize poses and landmarks with map frame values. */
         _mapper.insertOrientedPlaneFactors(currentPoseId);
         _mapper.setOrientedPlaneInitialValues();

         /* Optimize the Factor Graph. */
         _mapper.optimize();

         renderSLAMOutput();

         if (_mapper.ISAM2)
            _mapper.fgSLAM->clearISAM2();
      } else
      {
//         _mapper.poses.emplace_back(RigidBodyTransform(_mapper._sensorToMapTransform));
         //      _mesher.generateRegionLineMesh(_mapper.regionsInMapFrame, latestRegionEdges, frameIndex, _world);
      }

      ROS_DEBUG("Recycling Region Lists.\n");
      /* Load previous and current regions. Separated by SKIP_REGIONS. */
      _mapper.regions = _mapper._latestRegionsZUp;
      poseAvailable = false;

      ROS_DEBUG("SLAM Update Complete.\n");
   }
}

void SLAMModule::fileSLAMUpdate()
{
   AppUtils::getFileNames(_mapper.directory, _mapper.files, true);
   GeomTools::loadRegions(_frameId, fileRegions, _mapper.directory, _mapper.files);

   Vector3d position;
   Quaterniond orientation;

   cout << _mapper.directory + "poses.txt" << endl;
   ifstream fs(_mapper.directory + "poses.txt");
   GeomTools::loadPoseStamped(fs, position, orientation);

   _frameId++;

   cout << "File SLAM End" << endl;

   //   slamUpdate(fileRegions);

}

void SLAMModule::renderSLAMOutput()
{
   /* Generate World Frame Region Mesh. */
//      for (int k = 0; k < _mapper.regionsInMapFrame.size(); k++)
//         _mapper.regionsInMapFrame[k]->retainConvexHull();
//      _mesher.generateRegionLineMesh(_mapper.regionsInMapFrame, latestRegionEdges, 1, _world, false);

   /* Extracting Landmarks from Factor Graph. */
   _mapper.extractFactorGraphLandmarks();

   /* Reducing Vertex Count on Region Boundaries. */
   for (int k = 0; k < _mapper.mapRegions.size(); k++)
      _mapper.mapRegions[k]->retainConvexHull();

   /* Generating Region Mesh */
   _mesher.generateRegionLineMesh(_mapper.mapRegions, latestRegionEdges, _mapper.fgSLAM->getPoseId(), _world);

   /* Generating Pose Mesh */
   _mapper.fgSLAM->getPoses(_mapper.poses);

   _mapper.atlasPoses.emplace_back(_mapper._atlasSensorPose);
   _mesher.generatePoseMesh(_mapper.atlasPoses, atlasPoseAxes, _world, 3, 1.5);
//   _mesher.appendPoseMesh(_mapper._atlasSensorPose, atlasPoseAxes, _world, 3);
   _mesher.generatePoseMesh(_mapper.poses, poseAxes, _world, 2, 1.5);

//   _mesher.generateRegionLineMesh(_mapper._latestRegionsZUp, latestRegionEdges, 3, _world, true);
}

void SLAMModule::ImGuiUpdate()
{
   ImGui::Checkbox("SLAM Enabled", &this->enabled);
   ImGui::Checkbox("ICP Enabled", &this->ICPEnabled);
   ImGui::SliderFloat("Interpolation", &_interp, 0.0f, 1.0f);

   _mesher.generatePoseMesh(_mapper.atlasPoses, atlasPoseAxes, _world, 3, 1.5, _interp);

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

void SLAMModule::sensorPoseCallback(const geometry_msgs::PoseStampedConstPtr& poseMsg)
{
   if (_sensorPoseMessage == nullptr)
   {
      _sensorPoseMessage = poseMsg;
      return;
   }

   if (!poseAvailable)
   {

      Vector3d position;
      Vector3d previous;

      position << poseMsg->pose.position.x, poseMsg->pose.position.y, poseMsg->pose.position.z;
      previous << this->_sensorPoseMessage->pose.position.x, this->_sensorPoseMessage->pose.position.y, this->_sensorPoseMessage->pose.position.z;

      this->_sensorPoseMessage = poseMsg;

      double diff = (position - previous).norm();

//      if(diff > 0.001)
      {
         ROS_DEBUG("Distance From Last Transform: (%.4lf)\n", diff);

         this->_mapper._atlasSensorPose.setRotationAndTranslation(
               Eigen::Quaterniond(this->_sensorPoseMessage->pose.orientation.w, this->_sensorPoseMessage->pose.orientation.x,
                                  this->_sensorPoseMessage->pose.orientation.y, this->_sensorPoseMessage->pose.orientation.z),
               Eigen::Vector3d(this->_sensorPoseMessage->pose.position.x, this->_sensorPoseMessage->pose.position.y, this->_sensorPoseMessage->pose.position.z));

//         this->_mapper._atlasSensorPose.rotateX(-M_PI_2);
//         this->_mapper._atlasSensorPose.rotateY(-M_PI_2);
         //      this->_mapper._atlasSensorPose.rotateZ(M_PI);

         poseAvailable = true;

         //   this->_mapper._atlasSensorPose.print();
      }
   }

}
