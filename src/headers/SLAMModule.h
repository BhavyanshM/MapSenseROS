//
// Created by quantum on 5/28/21.
//

#ifndef SLAMMODULE_H
#define SLAMMODULE_H

#include "PlanarRegionMapHandler.h"
#include "MapsenseHeaders.h"

using namespace chrono;

class SLAMModule
{
   private:

      vector<shared_ptr<PlanarRegion>> fileRegions;

      vector<int> matchCountVec;
//      Subscriber *sensorPoseSub;
//      geometry_msgs::PoseStampedConstPtr _sensorPoseMessage;
      int _frameId = 0;

      RigidBodyTransform _transformZUp;

   public:
//      NetworkManager* network;
      PlanarRegionMapHandler _mapper;

      float _interp = 0.0f;
      bool enabled = true;
      bool initial = true;
      bool ICPEnabled = false;
      bool poseAvailable = false;

   public:
      SLAMModule(int argc, char **argv);

      void slamUpdate();

      void setLatestRegionsToZUp(const vector<shared_ptr<PlanarRegion>>& regions);

      void init(gtsam::Pose3 initialPose);

      void extractArgs(int argc, char** argv);

      void ImGuiUpdate();

//      void sensorPoseCallback(const geometry_msgs::PoseStampedConstPtr& poseMsg);

      void renderSLAMOutput();

      void SLAMTesterUpdate();
};

#endif //SLAMMODULE_H
