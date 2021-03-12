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
   graph.add(PriorFactor<Pose3>(poseIndex, mean, priorNoise));
}

void FactorGraphSLAM::addOdometryFactor(Pose3 odometry)
{
   graph.add(BetweenFactor<Pose3>(poseIndex, poseIndex + 1, odometry, odometryNoise));
   poseIndex++;
}

void FactorGraphSLAM::addOrientedPlaneLandmarkFactor(Vector4 lmMean, int lmIndex)
{
   graph.add(OrientedPlane3Factor(lmMean, orientedPlaneNoise, poseIndex, lmIndex));
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

