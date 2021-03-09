//
// Created by quantum on 2/24/21.
//

#ifndef SLAMAPPLICATION_H
#define SLAMAPPLICATION_H

#include "MagnumApplication.h"
#include "PlanarRegionMapTester.h"
#include <chrono>
#include "gtsam/geometry/Pose2.h"

using namespace std;
using namespace chrono;
using namespace gtsam;

class SLAMApplication : public MagnumApplication
{
   public:
      Pose2 pose;
      const int SKIP_EDGES = 3;
      const int SKIP_REGIONS = 1;
      int count = 0;
      int frameIndex = 0;
      vector<Object3D *> regionEdges, previousRegionEdges, matchingEdges;
      PlanarRegionMapHandler mapper;

      SLAMApplication(const Arguments& arguments);

      void tickEvent() override;

      void keyPressEvent(KeyEvent& event) override;

      void draw() override;

      void generateRegionLineMesh(vector<shared_ptr<PlanarRegion>> planarRegionList, vector<Object3D*>& regionEdges, int color);

      void generateMatchLineMesh(PlanarRegionMapHandler mapper, vector<Object3D*>& edges);

      void init(const Arguments& arguments);
};

#endif //SLAMAPPLICATION_H
