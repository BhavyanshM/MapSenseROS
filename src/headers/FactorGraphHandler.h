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
#include <bits/stdc++.h>

#define _DEBUG 1

#if _DEBUG == 1
   #define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)
#elif _DEBUG == 0
   #define LOG(x, ...)
#endif

using namespace boost::assign;

class FactorGraphHandler
{
   public:
      int getPoseId() const;

      FactorGraphHandler();

      //      void getPoses(std::vector<RigidBodyTransform>& poses);

      void AddPriorPoseFactor(int index, gtsam::Pose3 mean);

      void AddOdometryFactor(gtsam::Pose3 odometry, int poseId);

      void AddOrientedPlaneFactor(gtsam::Vector4 lmMean, int lmId, int poseIndex);

      void optimize();

      void OptimizeISAM2(uint8_t numberOfUpdates);

      void ClearISAM2();

      void SetPoseInitialValue(int index, gtsam::Pose3 value);

      void SetOrientedPlaneInitialValue(int landmarkId, gtsam::OrientedPlane3 value);

      const gtsam::Values& GetResults() const {return result;};

      const gtsam::Values& GetInitialValues() const {return initial;};

      const gtsam::NonlinearFactorGraph& GetFactorGraph();

      void createOdometryNoiseModel(gtsam::Vector6 odomVariance);

      void createOrientedPlaneNoiseModel(gtsam::Vector3 lmVariances);

      void incrementPoseId();

      void SLAMTest();

   private:
      gtsam::ISAM2Params parameters;

      gtsam::ISAM2 isam;
      std::unordered_set<std::string> structure;
      gtsam::Values initial, result;
      gtsam::NonlinearFactorGraph graph;
      gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
      gtsam::noiseModel::Diagonal::shared_ptr priorNoise2;
      gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;
      gtsam::noiseModel::Diagonal::shared_ptr orientedPlaneNoise;

};

#endif //FACTORGRAPHSLAM_H
