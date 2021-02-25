//
// Created by quantum on 2/24/21.
//

#ifndef PLOTTER3D_PY_SLAMAPPLICATION_H
#define PLOTTER3D_PY_SLAMAPPLICATION_H

#include "MagnumApplication.h"

class SLAMApplication : public MagnumApplication
{
   public:
      int count = 0;
      vector<Object3D *> regionEdges;
      PlanarRegionMapHandler mapper;

      SLAMApplication(const Arguments& arguments);

      void tickEvent() override;

      void draw() override;

      void draw_regions(vector<shared_ptr<PlanarRegion>> planarRegionList);
};

#endif //PLOTTER3D_PY_SLAMAPPLICATION_H
