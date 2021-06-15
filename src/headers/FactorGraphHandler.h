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
#include <bits/stdc++.h>

#define _DEBUG 0

#if _DEBUG == 1
   #define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)
#elif _DEBUG == 0
   #define LOG(x, ...)
#endif

using namespace boost::assign;
using namespace gtsam;

class FactorGraphHandler
{
   private:

      ISAM2Params parameters;
      ISAM2 isam;

      std::unordered_set<std::string> structure;
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

      FactorGraphHandler();

      void getPoses(std::vector<RigidBodyTransform>& poses);

      int addPriorPoseFactor(Pose3 mean);

      int addOdometryFactor(Pose3 odometry);

      int addOrientedPlaneLandmarkFactor(Vector4 lmMean, int lmId, int poseIndex);

      void optimize();

      void optimizeISAM2(uint8_t numberOfUpdates);

      void clearISAM2();

      void setPoseInitialValue(int index, Pose3 value);

      void setOrientedPlaneInitialValue(int index, OrientedPlane3 value);

      Values getResults();

      Values getInitial();

      NonlinearFactorGraph getFactorGraph();

      void createOdometryNoiseModel(Vector6 odomVariance);

      void createOrientedPlaneNoiseModel(Vector3 lmVariances);

      void incrementPoseId();
};

#endif //FACTORGRAPHSLAM_H
