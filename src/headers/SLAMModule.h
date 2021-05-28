//
// Created by quantum on 5/28/21.
//

#ifndef SLAMMODULE_H
#define SLAMMODULE_H

#include "PlanarRegionMapHandler.h"
#include "MeshGenerator.h"
#include <chrono>

using namespace chrono;

class SLAMModule
{
   private:
      vector<Object3D *> latestRegionEdges, previousRegionEdges, matchingEdges, poseAxes;

      PlanarRegionMapHandler _mapper;
//      MeshGenerator _mesher;

   public:
      SLAMModule(int argc, char** argv);

      void slamUpdate(vector<shared_ptr<PlanarRegion>> latestRegions);

      void init();
};

#endif //SLAMMODULE_H
