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
   public:

      float _interp = 0.0f;
      bool enabled = true;
      bool initial = true;
      bool ICPEnabled = true;
      bool poseAvailable = false;

      SLAMModule(int argc, char **argv);

      void SetMapHandler(PlanarRegionMapHandler* mapHandler) { _mapper = mapHandler;}

      void Initialize(vector<shared_ptr<PlanarRegion>>& regions);

      void Update(vector<shared_ptr<PlanarRegion>>& regions);

      void setLatestRegionsToZUp(const vector<shared_ptr<PlanarRegion>>& regions);

      void init(gtsam::Pose3 initialPose);

      void extractArgs(int argc, char** argv);

      void ImGuiUpdate();

//      void sensorPoseCallback(const geometry_msgs::PoseStampedConstPtr& poseMsg);

      void renderSLAMOutput();

      void SLAMTesterUpdate();

   private:
      PlanarRegionMapHandler* _mapper;
      vector<shared_ptr<PlanarRegion>> fileRegions;

      vector<int> matchCountVec;
      int _frameId = 0;

      RigidBodyTransform _transformZUp;
      bool render = true;

      int count = 0;
};

#endif //SLAMMODULE_H
