#include "FactorGraphSLAM.h"

FactorGraphSLAM::FactorGraphSLAM()
{
   /* Set ISAM2 parameters here. */
   this->isam = ISAM2(parameters);

   Vector6 odomVariance;
   odomVariance << 1e-2, 1e-2, 1e-2, 1e-1, 1e-1, 1e-1;
   createOdometryNoiseModel(odomVariance);

   Vector3 lmVariance;
   lmVariance << 1e-2, 1e-2, 1e-2;
   createOrientedPlaneNoiseModel(lmVariance);

   Vector6 priorVariance;
//   priorVariance << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
   createPriorPoseNoiseModel(priorVariance);

}

void FactorGraphSLAM::getPoses(std::vector<RigidBodyTransform>& poses)
{
   for (int i = 1; i < this->getPoseId(); i++)
   {
      RigidBodyTransform mapToSensorTransform(this->getResults().at<Pose3>(Symbol('x', i)).matrix());
      poses.emplace_back(mapToSensorTransform.getInverse());
   }
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
   LOG("Inserting Prior Pose: Pose(%d)\n", poseId);
   graph.add(PriorFactor<Pose3>(Symbol('x', poseId), mean, priorNoise));
   return poseId;
}

int FactorGraphSLAM::addOdometryFactor(Pose3 odometry)
{
   LOG("Inserting Odometry: Pose(%d) Pose(%d)\n", poseId, poseId + 1);
   graph.add(BetweenFactor<Pose3>(Symbol('x', poseId), Symbol('x', poseId + 1), odometry, odometryNoise));
   poseId++;
   return poseId;
}

int FactorGraphSLAM::addOrientedPlaneLandmarkFactor(Vector4 lmMean, int lmId)
{
   int landmarkId = (lmId != -1) ? lmId : newLandmarkId++;
   LOG("Inserting Oriented Plane Factor: Pose(%d) Landmark(%d, %d)\n", poseId, lmId, landmarkId);
   graph.add(OrientedPlane3Factor(lmMean, orientedPlaneNoise, Symbol('x', poseId), Symbol('l', landmarkId)));
   return landmarkId;
}

void FactorGraphSLAM::initPoseValue(Pose3 value)
{
   LOG("Initializing Pose:(%d)\n", poseId);
   initial.insert(Symbol('x', poseId), value);
}

void FactorGraphSLAM::initOrientedPlaneLandmarkValue(int index, OrientedPlane3 value)
{
   LOG("Initializing Oriented Plane Landmark:(%d)\n", index);
   if (!initial.exists(Symbol('l', index)))
   {
      initial.insert(Symbol('l', index), value);
   }
}

void FactorGraphSLAM::optimize()
{
   LOG("Optimizing\n");
   result = LevenbergMarquardtOptimizer(graph, initial).optimize();
}

void FactorGraphSLAM::optimizeISAM2()
{
   LOG("Optimizing Using ISAM2.\n");
   isam.update(graph, initial);
   result = isam.calculateEstimate();
}

void FactorGraphSLAM::clearISAM2()
{
   graph.resize(0);
   initial.clear();
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

int FactorGraphSLAM::getPoseId() const
{
   return poseId;
}

