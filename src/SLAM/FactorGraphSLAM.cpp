#include "FactorGraphSLAM.h"

FactorGraphSLAM::FactorGraphSLAM()
{

   Vector6 odomVariance;
   odomVariance << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
   createOdometryNoiseModel(odomVariance);

   Vector3 lmVariance;
   lmVariance << 0.1, 0.1, 0.1;
   createOrientedPlaneNoiseModel(lmVariance);

   Vector6 priorVariance;
   odomVariance << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
   createPriorPoseNoiseModel(priorVariance);

   addPriorPoseFactor(Pose3().identity());
}

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

int FactorGraphSLAM::addOdometryFactor(Pose3 odometry)
{
   graph.add(BetweenFactor<Pose3>(Symbol('x', poseId), Symbol('x', poseId), odometry, odometryNoise));
   poseId++;
   return poseId;
}

int FactorGraphSLAM::addOrientedPlaneLandmarkFactor(Vector4 lmMean, int lmId)
{
   int landmarkId = (lmId != -1) ? lmId : newLandmarkId++;
   graph.add(OrientedPlane3Factor(lmMean, orientedPlaneNoise, Symbol('x', poseId), Symbol('l', newLandmarkId)));
   return landmarkId;
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

