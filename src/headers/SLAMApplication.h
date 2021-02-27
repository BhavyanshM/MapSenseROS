//
// Created by quantum on 2/24/21.
//

#ifndef PLOTTER3D_PY_SLAMAPPLICATION_H
#define PLOTTER3D_PY_SLAMAPPLICATION_H

#include "MagnumApplication.h"

class SLAMApplication : public MagnumApplication
{
   public:
      const int SKIP_EDGES = 16;
      int count = 0;
      vector<Object3D *> regionEdges;
      PlanarRegionMapHandler mapper;
      vector<shared_ptr<PlanarRegion>> regions, previousRegions;

      SLAMApplication(const Arguments& arguments);

      void tickEvent() override;

      void draw() override;

      void draw_regions(vector<shared_ptr<PlanarRegion>> planarRegionList, int color);
};

#endif //PLOTTER3D_PY_SLAMAPPLICATION_H
