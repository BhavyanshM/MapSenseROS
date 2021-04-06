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

#include "RigidBodyTransform.h"

#include <boost/bind.hpp>
#include <boost/assign/std/vector.hpp>

#define _DEBUG 1

#if _DEBUG == 1
   #define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)
#elif _DEBUG == 0
   #define LOG(x)
#endif

using namespace boost::assign;
using namespace gtsam;

class FactorGraphSLAM
{
   private:

      ISAM2Params parameters;
      ISAM2 isam;

      Values initial, result;
      NonlinearFactorGraph graph;
      noiseModel::Diagonal::shared_ptr priorNoise;
      noiseModel::Diagonal::shared_ptr priorNoise2;
      noiseModel::Diagonal::shared_ptr odometryNoise;
      noiseModel::Diagonal::shared_ptr orientedPlaneNoise;
      int poseId = 1;
   public:
      int getPoseId() const;

   private:
      int newLandmarkId = 1;

   public:

      FactorGraphSLAM();

      void getPoses(std::vector<RigidBodyTransform>& poses);

      int addPriorPoseFactor(Pose3 mean, int poseId);

      int addOdometryFactor(Pose3 odometry);

      int addOrientedPlaneLandmarkFactor(Vector4 lmMean, int lmId);

      void optimize();

      void optimizeISAM2();

      void clearISAM2();

      void initPoseValue(Pose3 value);

      void initOrientedPlaneLandmarkValue(int index, OrientedPlane3 value);

      Values getResults();

      Values getInitial();

      NonlinearFactorGraph getFactorGraph();

      void createOdometryNoiseModel(Vector6 odomVariance);

      void createOrientedPlaneNoiseModel(Vector3 lmVariances);
};

#endif //FACTORGRAPHSLAM_H
