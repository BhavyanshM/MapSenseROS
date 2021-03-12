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
      noiseModel::Diagonal::shared_ptr priorNoise;
      noiseModel::Diagonal::shared_ptr odometryNoise;
      noiseModel::Diagonal::shared_ptr orientedPlaneNoise;

   public:
      void addPriorPoseFactor(Pose3 mean);

      void addOdometryFactor(Pose3 odometry);

      void addOrientedPlaneLandmarkFactor(Vector4 lmMean, int lmIndex);

      void optimize();

      void initPoseValue(int index, Pose3 value);

      void initOrientedPlaneLandmarkValue(int index, OrientedPlane3 value);

      Values getResults();

      void createPriorPoseNoiseModel(Vector6 variance);

      void createOdometryNoiseModel(Vector6 odomVariance);

      void createOrientedPlaneNoiseModel(Vector3 lmVariances);
};

#endif //FACTORGRAPHSLAM_H
