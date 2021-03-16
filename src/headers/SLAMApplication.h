//
// Created by quantum on 2/24/21.
//

#ifndef SLAMAPPLICATION_H
#define SLAMAPPLICATION_H

#include "MagnumApplication.h"
#include "PlanarRegionMapTester.h"
#include <chrono>
#include "gtsam/geometry/Pose2.h"
#include "../SLAM/FactorGraphSLAM.h"

using namespace std;
using namespace chrono;

class SLAMApplication : public MagnumApplication
{
   public:
      const int SKIP_EDGES = 3;
      const int SKIP_REGIONS = 1;
      int count = 0;
      int frameIndex = 0;
      vector<Object3D *> regionEdges, previousRegionEdges, matchingEdges;
      PlanarRegionMapHandler mapper;
      Object3D& frameOrigin = _sensor->addChild<Object3D>();
      Eigen::MatrixXd sensorPoseWorldFrame, sensorPoseRelative = Eigen::MatrixXd::Identity(4,4);
      FactorGraphSLAM fgSLAM;

      SLAMApplication(const Arguments& arguments);

      void tickEvent() override;

      void keyPressEvent(KeyEvent& event) override;

      void draw() override;

      void generateRegionLineMesh(vector<shared_ptr<PlanarRegion>> planarRegionList, vector<Object3D *>& regionEdges, int color);

      void generateMatchLineMesh(PlanarRegionMapHandler mapper, vector<Object3D *>& edges);

      void init(const Arguments& arguments);

      void updateFactorGraph();

};

#endif //SLAMAPPLICATION_H
