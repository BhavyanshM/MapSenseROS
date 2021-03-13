#include "FactorGraphSLAM.h"

void FactorGraphSLAM::createPriorPoseNoiseModel(Vector6 variance)
{
   priorNoise = noiseModel::Diagonal::Variances(variance);
}

void FactorGraphSLAM::createOdometryNoiseModel(Vector6 odomVariance)
{
   odometryNoise = noiseModel::Diagonal::Variances(odomVariance);
}

void FactorGraphSLAM::createOrientedPlaneNoiseModel(Vector3 lmVariances)
{
   orientedPlaneNoise = noiseModel::Diagonal::Variances(lmVariances);
}

void FactorGraphSLAM::addPriorPoseFactor(Pose3 mean)
{
   graph.add(PriorFactor<Pose3>(poseId, mean, priorNoise));
}

void FactorGraphSLAM::addOdometryFactor(Pose3 odometry)
{
   poseId += newLandmarkId + 1;
   graph.add(BetweenFactor<Pose3>(poseId, poseId + 1, odometry, odometryNoise));
   poseId++;
   newLandmarkId = 0;
}

int FactorGraphSLAM::addOrientedPlaneLandmarkFactor(Vector4 lmMean, int lmId)
{
   if (lmId != -1)
   {
      graph.add(OrientedPlane3Factor(lmMean, orientedPlaneNoise, poseId, lmId));
      return lmId;
   } else
   {
      newLandmarkId++;
      graph.add(OrientedPlane3Factor(lmMean, orientedPlaneNoise, poseId, poseId + newLandmarkId));
      return poseId + newLandmarkId;
   }
}

void FactorGraphSLAM::generateNextPoseId(int numberOfLandmarks)
{
   poseId += numberOfLandmarks + 1;
}

void FactorGraphSLAM::initPoseValue(int index, Pose3 value)
{
   initial.insert(index, value);
}

void FactorGraphSLAM::initOrientedPlaneLandmarkValue(int index, OrientedPlane3 value)
{
   initial.insert(index, value);
}

void FactorGraphSLAM::optimize()
{
   result = LevenbergMarquardtOptimizer(graph, initial).optimize();
}

Values FactorGraphSLAM::getResults()
{
   return result;
}

