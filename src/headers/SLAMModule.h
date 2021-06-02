//
// Created by quantum on 5/28/21.
//

#ifndef SLAMMODULE_H
#define SLAMMODULE_H

#include "PlanarRegionMapHandler.h"
#include "MeshGenerator.h"
#include <chrono>
#include "imgui.h"
#include "implot/implot.h"

using namespace chrono;

class SLAMModule
{
   private:
      vector<Object3D *> latestRegionEdges, previousRegionEdges, matchingEdges, poseAxes;
      vector<int> matchCountVec;

   public:
      PlanarRegionMapHandler _mapper;
//      MeshGenerator _mesher;

      bool enabled = false;
   public:
      SLAMModule(int argc, char** argv);

      void slamUpdate(vector<shared_ptr<PlanarRegion>> latestRegions);

      void init();

      void extractArgs(int argc, char** argv);

      void ImGuiUpdate();
};

#endif //SLAMMODULE_H
