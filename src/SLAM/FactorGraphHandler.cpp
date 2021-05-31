#include "FactorGraphHandler.h"

FactorGraphHandler::FactorGraphHandler()
{
   /* Set ISAM2 parameters here. */
   parameters.relinearizeThreshold = 0.01;
   parameters.relinearizeSkip = 1;
   this->isam = ISAM2(parameters);

   Vector6 odomVariance;
   odomVariance << 1e-2, 1e-2, 1e-2, 1e-1, 1e-1, 1e-1;
   createOdometryNoiseModel(odomVariance);

   Vector3 lmVariance;
   lmVariance << 1e-2, 1e-2, 1e-2;
   createOrientedPlaneNoiseModel(lmVariance);

   Vector6 priorVariance;
   priorVariance << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
   priorNoise = noiseModel::Diagonal::Variances(priorVariance);

   Vector6 priorVariance2;
   priorVariance2 << 1e2, 1e2, 1e2, 1e2, 1e2, 1e2;
   priorNoise2 = noiseModel::Diagonal::Variances(priorVariance2);
}

void FactorGraphHandler::getPoses(std::vector<RigidBodyTransform>& poses)
{
   poses.clear();
   for (int i = 1; i < this->getPoseId(); i++)
   {
      RigidBodyTransform mapToSensorTransform(this->getResults().at<Pose3>(Symbol('x', i)).matrix());
      poses.emplace_back(mapToSensorTransform);
   }
}

void FactorGraphHandler::createOdometryNoiseModel(Vector6 odomVariance)
{
   odometryNoise = noiseModel::Diagonal::Variances(odomVariance);
}

void FactorGraphHandler::createOrientedPlaneNoiseModel(Vector3 lmVariances)
{
   orientedPlaneNoise = noiseModel::Diagonal::Variances(lmVariances);
}

int FactorGraphHandler::addPriorPoseFactor(const Pose3& mean, const int& poseId)
{
   LOG("%s: Inserting Prior Pose: Pose(%d)\n", poseId, __func__);
   graph.add(PriorFactor<Pose3>(Symbol('x', poseId), mean, priorNoise));
   return poseId;
}

int FactorGraphHandler::addOdometryFactor(Pose3 odometry)
{
   LOG("%s: Inserting Odometry: Pose(%d) Pose(%d)\n", poseId, poseId + 1, __func__);
   graph.add(BetweenFactor<Pose3>(Symbol('x', poseId), Symbol('x', poseId + 1), odometry, odometryNoise));
   poseId++;
   return poseId;
}

int FactorGraphHandler::addOrientedPlaneLandmarkFactor(Vector4 lmMean, int lmId)
{
   int landmarkId = (lmId != -1) ? lmId : newLandmarkId++;
   LOG("%s: Inserting Oriented Plane Factor: Pose(%d) Landmark(%d, %d)\n", poseId, lmId, landmarkId, __func__);
   graph.add(OrientedPlane3Factor(lmMean, orientedPlaneNoise, Symbol('x', poseId), Symbol('l', landmarkId)));
   return landmarkId;
}

void FactorGraphHandler::initPoseValue(Pose3 value)
{
   if (structure.find('x' + std::to_string(poseId)) == structure.end())
   {
      LOG("%s: Initializing Pose:(%d)\n", poseId, __func__);
      structure.insert('x' + std::to_string(poseId));
      initial.insert(Symbol('x', poseId), value);
   }
}

void FactorGraphHandler::initOrientedPlaneLandmarkValue(int index, OrientedPlane3 value)
{
   if (!initial.exists(Symbol('l', index)) && structure.find('l' + std::to_string(index)) == structure.end())
   {
      LOG("%s: Initializing Oriented Plane Landmark:(%d)\n", index, __func__);
      structure.insert('l' + std::to_string(index));
      initial.insert(Symbol('l', index), value);
   }
}

void FactorGraphHandler::optimize()
{
   LOG("%s: Optimizing\n", __func__);
   result = LevenbergMarquardtOptimizer(graph, initial).optimize();
}

void FactorGraphHandler::optimizeISAM2(uint8_t numberOfUpdates)
{
   LOG("%s: Optimizing Graph Using ISAM2.\n", __func__);
   isam.update(graph, initial);
   for(uint8_t i = 1; i< numberOfUpdates; i++)
   {
      isam.update();
   }
   result = isam.calculateEstimate();
   LOG("%s: Optimization Successful.", __func__);
}

void FactorGraphHandler::clearISAM2()
{
   initial.clear();
   graph.resize(0);
}

Values FactorGraphHandler::getResults()
{
   return result;
}

Values FactorGraphHandler::getInitial()
{
   return initial;
}

NonlinearFactorGraph FactorGraphHandler::getFactorGraph()
{
   return graph;
}

int FactorGraphHandler::getPoseId() const
{
   return poseId;
}

