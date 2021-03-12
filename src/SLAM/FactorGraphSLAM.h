//
// Created by quantum on 3/9/21.
//

#ifndef FACTORGRAPHSLAM_H
#define FACTORGRAPHSLAM_H

#include <gtsam/slam/OrientedPlane3Factor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <CppUnitLite/TestHarness.h>
#include <boost/bind.hpp>
#include <boost/assign/std/vector.hpp>

using namespace boost::assign;
using namespace gtsam;
using namespace std;

class FactorGraphSLAM
{
   private:
      ISAM2 isam2;
      Values initial, result;
      NonlinearFactorGraph graph;
      int poseIndex = 1;

   public:
      void addPriorPoseFactor(Pose3 mean, Vector6 variance);

      void addOdometryFactor(Pose3 odometry, Vector6 odomVariance);

      void addOrientedPlaneLandmarkFactor(Vector4 lmMean, Vector4 lmVariances, int lmIndex);

      void optimize();

      void initPoseValue(int index, Pose3 value);

      void initOrientedPlaneLandmarkValue(int index, OrientedPlane3 value);

      Values getResults();
};

#endif //FACTORGRAPHSLAM_H
