//
// Created by quantum on 5/28/21.
//

#ifndef SLAMMODULE_H
#define SLAMMODULE_H

#include "PlanarRegionMapHandler.h"
#include "MeshGenerator.h"
#include "imgui.h"
#include "implot/implot.h"
#include "NetworkManager.h"
#include "MapsenseHeaders.h"

using namespace chrono;

class SLAMModule
{
   private:

      vector<shared_ptr<PlanarRegion>> fileRegions;

      vector<Object3D *> latestRegionEdges, previousRegionEdges, matchingEdges, poseAxes, atlasPoseAxes;
      vector<int> matchCountVec;
      Subscriber *sensorPoseSub;
      geometry_msgs::PoseStampedConstPtr _sensorPoseMessage;
      int _frameId = 0;

   public:
      Object3D* _world;
      NetworkManager* network;
      PlanarRegionMapHandler _mapper;
      MeshGenerator _mesher;

      bool enabled = true;
      bool initial = true;
      bool ICPEnabled = false;
      bool poseAvailable = false;

   public:
      SLAMModule(int argc, char **argv, NetworkManager *networkManager, Magnum::SceneGraph::DrawableGroup3D* _drawables, Object3D* sensor);

      void slamUpdate(const vector<shared_ptr<PlanarRegion>>& latestRegions);

      void init(Pose3 initialPose);

      void extractArgs(int argc, char** argv);

      void ImGuiUpdate();

      void sensorPoseCallback(const geometry_msgs::PoseStampedConstPtr& poseMsg);

      void renderSLAMOutput();

      void fileSLAMUpdate();
};

#endif //SLAMMODULE_H
