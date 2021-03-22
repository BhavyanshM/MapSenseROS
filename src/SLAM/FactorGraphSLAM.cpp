#include "FactorGraphSLAM.h"

FactorGraphSLAM::FactorGraphSLAM()
{

   Vector6 odomVariance;
   odomVariance << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2;
   createOdometryNoiseModel(odomVariance);

   Vector3 lmVariance;
   lmVariance << 1e-4, 1e-4, 1e-4;
   createOrientedPlaneNoiseModel(lmVariance);

   Vector6 priorVariance;
   odomVariance << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
   createPriorPoseNoiseModel(priorVariance);

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
   if (DEBUG == 2)
      printf("Inserting Prior Pose: Pose(%d)\n", poseId);
   graph.add(PriorFactor<Pose3>(Symbol('x', poseId), mean, priorNoise));
   return poseId;
}

int FactorGraphSLAM::addOdometryFactor(Pose3 odometry)
{
   if (DEBUG == 2)
      printf("Inserting Odometry: Pose(%d) Pose(%d)\n", poseId, poseId + 1);
   graph.add(BetweenFactor<Pose3>(Symbol('x', poseId), Symbol('x', poseId + 1), odometry, odometryNoise));
   poseId++;
   return poseId;
}

int FactorGraphSLAM::addOrientedPlaneLandmarkFactor(Vector4 lmMean, int lmId)
{
   int landmarkId = (lmId != -1) ? lmId : newLandmarkId++;
   if (DEBUG == 2)
      printf("Inserting Oriented Plane Factor: Pose(%d) Landmark(%d, %d)\n", poseId, lmId, landmarkId);
   graph.add(OrientedPlane3Factor(lmMean, orientedPlaneNoise, Symbol('x', poseId), Symbol('l', landmarkId)));
   return landmarkId;
}

void FactorGraphSLAM::initPoseValue(Pose3 value)
{
   if (DEBUG == 2)
      printf("Initializing Pose:(%d)\n", poseId);
   initial.insert(Symbol('x', poseId), value);
}

void FactorGraphSLAM::initOrientedPlaneLandmarkValue(int index, OrientedPlane3 value)
{
   if (!initial.exists(Symbol('l', index)))
   {
      if (DEBUG == 2)
         printf("Initializing Oriented Plane Landmark:(%d)\n", index);
      initial.insert(Symbol('l', index), value);
   }
}

void FactorGraphSLAM::optimize()
{
   if (DEBUG == 2)
      printf("Optimizing\n");
   result = LevenbergMarquardtOptimizer(graph, initial).optimize();
}

Values FactorGraphSLAM::getResults()
{
   return result;
}

Values FactorGraphSLAM::getInitial()
{
   return initial;
}

NonlinearFactorGraph FactorGraphSLAM::getFactorGraph()
{
   return graph;
}

