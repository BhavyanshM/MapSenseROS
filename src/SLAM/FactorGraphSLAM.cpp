#include "FactorGraphSLAM.h"

void FactorGraphSLAM::addPriorPoseFactor(Pose3 mean, Vector6 variance)
{
   noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(variance);
   graph.add(PriorFactor<Pose3>(poseIndex, mean, priorNoise));
}

void FactorGraphSLAM::addOdometryFactor(Pose3 odometry, Vector6 odomVariance)
{
   noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(odomVariance);
   graph.add(BetweenFactor<Pose3>(poseIndex, poseIndex + 1, odometry, odometryNoise));
   poseIndex++;
}

void FactorGraphSLAM::addOrientedPlaneLandmarkFactor(Vector4 lmMean, Vector4 lmVariances, int lmIndex)
{
   noiseModel::Diagonal::shared_ptr lmNoise = noiseModel::Diagonal::Variances(lmVariances);
   graph.add(OrientedPlane3Factor(lmMean, lmNoise, lmIndex, poseIndex));
}

void FactorGraphSLAM::initPoseValue(int index, Pose3 value){
   initial.insert(index, value);
}

void FactorGraphSLAM::initOrientedPlaneLandmarkValue(int index, OrientedPlane3 value){
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

