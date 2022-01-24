//
// Created by quantum on 1/22/22.
//

#ifndef MAP_SENSE_BUNDLEADJUSTMENT_H
#define MAP_SENSE_BUNDLEADJUSTMENT_H

// Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// We want to use iSAM2 to solve the structure-from-motion problem incrementally, so
// include iSAM2 here
#include <gtsam/nonlinear/ISAM2.h>

// iSAM2 requires as input a set of new factors to be added stored in a factor graph,
// and initial guesses for any new variables used in the added factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Projection factors to model the camera's landmark observations.
// Also, we will initialize the robot at some location using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/SimpleCamera.h>

#include "PointLandmark.h"
#include "FactorGraphHandler.h"
#include "CameraParams.h"

class BundleAdjustment
{
   public:
      BundleAdjustment(CameraParams& params);

      void InsertProjectionFactor();

      void InsertPosePriorFactor();

      void Optimize();

      void Initialize();

      void ComputeNonLinear(std::vector<gtsam::Point2>& measurements, std::vector<gtsam::Pose3>& poses, std::vector<gtsam::Point3>& points);

      void Update(std::vector<PointLandmark>& landmarks, std::vector<Eigen::Matrix4f>& eigenPoses);

   private:
//      FactorGraphHandler *fgHandle;
//
//      gtsam::noiseModel::Isotropic::shared_ptr cameraMeasurementNoise;
//      gtsam::ISAM2Params parameters;
//      gtsam::ISAM2 isam;
//      gtsam::NonlinearFactorGraph graph;
//
//      gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;
//      std::unordered_set<std::string> structure;
//      gtsam::Values initial, result;
//      gtsam::Cal3_S2::shared_ptr K;
};

#endif //MAP_SENSE_BUNDLEADJUSTMENT_H
