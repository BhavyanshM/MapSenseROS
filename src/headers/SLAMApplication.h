//
// Created by quantum on 2/24/21.
//

#ifndef PLOTTER3D_PY_SLAMAPPLICATION_H
#define PLOTTER3D_PY_SLAMAPPLICATION_H

#include "MagnumApplication.h"

class SLAMApplication : public MagnumApplication
{
   public:
      const int SKIP_EDGES = 3;
      int count = 0;
      int frameIndex = 1;
      vector<Object3D *> regionEdges, previousRegionEdges;
      PlanarRegionMapHandler mapper;
      vector<shared_ptr<PlanarRegion>> regions, previousRegions;

      SLAMApplication(const Arguments& arguments);

      void tickEvent() override;

      void keyPressEvent(KeyEvent& event) override;

      void draw() override;

      void generateRegionLineMesh(vector<shared_ptr<PlanarRegion>> planarRegionList, vector<Object3D*>& regionEdges, int color);
};

#endif //PLOTTER3D_PY_SLAMAPPLICATION_H
