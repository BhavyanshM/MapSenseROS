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
      noiseModel::Diagonal::shared_ptr priorNoise;
      noiseModel::Diagonal::shared_ptr odometryNoise;
      noiseModel::Diagonal::shared_ptr orientedPlaneNoise;
      int poseId = 1;
      int newLandmarkId = 1;

   public:

      FactorGraphSLAM();

      int addPriorPoseFactor(Pose3 mean);

      int addOdometryFactor(Pose3 odometry);

      int addOrientedPlaneLandmarkFactor(Vector4 lmMean, int lmId);

      void optimize();

      void initPoseValue(Pose3 value);

      void initOrientedPlaneLandmarkValue(int index, OrientedPlane3 value);

      Values getResults();

      void createPriorPoseNoiseModel(Vector6 variance);

      void createOdometryNoiseModel(Vector6 odomVariance);

      void createOrientedPlaneNoiseModel(Vector3 lmVariances);

      void generateNextPoseId(int numberOfLandmarks);
};

#endif //FACTORGRAPHSLAM_H
