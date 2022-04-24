//
// Created by quantum on 5/28/21.
//

#ifndef SLAMMODULE_H
#define SLAMMODULE_H

class PlanarRegion;

#include "vector"
#include "memory"

class SLAMModule
{
   private:

      std::vector<std::shared_ptr<PlanarRegion>> fileRegions;

      std::vector<int> matchCountVec;
//      Subscriber *sensorPoseSub;
//      geometry_msgs::PoseStampedConstPtr _sensorPoseMessage;
      int _frameId = 0;


      bool render = true;

   public:
//      NetworkManager* network;


      float _interp = 0.0f;
      bool enabled = true;
      bool initial = true;
      bool ICPEnabled = false;
      bool poseAvailable = false;

   public:
      SLAMModule(int argc, char **argv);

      void Update(const std::vector <std::shared_ptr<PlanarRegion>>& regions);

      void setLatestRegionsToZUp(const std::vector<std::shared_ptr<PlanarRegion>>& regions);

//      void init(gtsam::Pose3 initialPose);

//      void extractArgs(int argc, char** argv);

      void ImGuiUpdate();

//      void sensorPoseCallback(const geometry_msgs::PoseStampedConstPtr& poseMsg);

      void renderSLAMOutput();

      void SLAMTesterUpdate();
};

#endif //SLAMMODULE_H
