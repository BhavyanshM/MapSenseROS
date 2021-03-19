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

int FactorGraphSLAM::addPriorPoseFactor(Pose3 mean)
{
   printf("Inserting Prior Pose: Pose(%d)\n", poseId);
   graph.add(PriorFactor<Pose3>(Symbol('x', poseId), mean, priorNoise));
   return poseId;
}

int FactorGraphSLAM::addOdometryFactor(Pose3 odometry)
{
   printf("Inserting Odometry: Pose(%d) Pose(%d)\n", poseId, poseId + 1);
   graph.add(BetweenFactor<Pose3>(Symbol('x', poseId), Symbol('x', poseId + 1), odometry, odometryNoise));
   poseId++;
   return poseId;
}

int FactorGraphSLAM::addOrientedPlaneLandmarkFactor(Vector4 lmMean, int lmId)
{
   int landmarkId = (lmId != -1) ? lmId : newLandmarkId++;
   printf("Inserting Oriented Plane Factor: Pose(%d) Landmark(%d, %d)\n", poseId, lmId, landmarkId);
   graph.add(OrientedPlane3Factor(lmMean, orientedPlaneNoise, Symbol('x', poseId), Symbol('l', newLandmarkId)));
   return landmarkId;
}

void FactorGraphSLAM::initPoseValue(Pose3 value)
{
   printf("Initializing Pose:(%d)\n", poseId);
   initial.insert(Symbol('x', poseId), value);
}

void FactorGraphSLAM::initOrientedPlaneLandmarkValue(int index, OrientedPlane3 value)
{
   printf("Initializing Oriented Plane Landmark:(%d)\n", index);
   initial.insert(Symbol('l', index), value);
}

void FactorGraphSLAM::optimize()
{
   printf("Optimizing\n");
   result = LevenbergMarquardtOptimizer(graph, initial).optimize();
}

Values FactorGraphSLAM::getResults()
{
   printf("Getting Results\n");
   return result;
}

