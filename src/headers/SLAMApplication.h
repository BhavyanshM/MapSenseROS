//
// Created by quantum on 2/24/21.
//

#ifndef SLAMAPPLICATION_H
#define SLAMAPPLICATION_H

#include "MagnumApplication.h"
#include "PlanarRegionMapTester.h"
#include "gtsam/geometry/Pose2.h"
#include "../SLAM/FactorGraphSLAM.h"
#include "MeshGenerator.h"
#include "AppUtils.h"
#include "StructureFromMotion.h"
#include "MapsenseHeaders.h"

using namespace std;
using namespace chrono;

class SLAMApplication : public MagnumApplication
{
   public:

      const int SKIP_REGIONS = 1;
      int count = 0;
      int frameIndex = 0;
      vector<Object3D *> latestRegionEdges, previousRegionEdges, matchingEdges, poseAxes;
      Object3D& frameOrigin = _sensor->addChild<Object3D>();

      MeshGenerator _mesher;
      PlanarRegionMapHandler _mapper;

      SLAMApplication(const Arguments& arguments);

      void tickEvent() override;

      void keyPressEvent(KeyEvent& event) override;

      void draw() override;

      void init(const Arguments& arguments);

      void slamUpdate(vector<shared_ptr<PlanarRegion>> regionsInMapFrame);

};

#endif //SLAMAPPLICATION_H
