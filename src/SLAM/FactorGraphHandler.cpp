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
   printf("getPoses()");
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

int FactorGraphHandler::addPriorPoseFactor(Pose3 mean)
{
   LOG("addPriorPoseFactor(x%d)\n", poseId);
   graph.add(PriorFactor<Pose3>(Symbol('x', poseId), mean, priorNoise));
   return poseId;
}

int FactorGraphHandler::addOdometryFactor(Pose3 odometry)
{
   LOG("addOdometryFactor(x%d -o- x%d)\n", poseId, poseId + 1);
   graph.add(BetweenFactor<Pose3>(Symbol('x', poseId), Symbol('x', poseId + 1), odometry, odometryNoise));
   poseId++;
   return poseId;
}

int FactorGraphHandler::addOrientedPlaneLandmarkFactor(Vector4 lmMean, int lmId, int poseIndex)
{
   int landmarkId = (lmId != -1) ? lmId : newLandmarkId++;
   LOG("addOrientedPlaneLandmarkFactor(x%d -o- l%d)\n", poseIndex, landmarkId);
   graph.add(OrientedPlane3Factor(lmMean, orientedPlaneNoise, Symbol('x', poseIndex), Symbol('l', landmarkId)));
   return landmarkId;
}

void FactorGraphHandler::setPoseInitialValue(int index, Pose3 value)
{
   LOG("setPoseInitialValue(x%d)\n", index);
   if (structure.find('x' + std::to_string(index)) == structure.end())
   {
      structure.insert('x' + std::to_string(index));
      initial.insert(Symbol('x', index), value);
   }
}

void FactorGraphHandler::setOrientedPlaneInitialValue(int index, OrientedPlane3 value)
{
   LOG("setOrientedPlaneInitialValue(l%d)\n", index);
   if (!initial.exists(Symbol('l', index)) && structure.find('l' + std::to_string(index)) == structure.end())
   {
      structure.insert('l' + std::to_string(index));
      initial.insert(Symbol('l', index), value);
   }
}

void FactorGraphHandler::optimize()
{
   LOG("optimize()\n");
   result = LevenbergMarquardtOptimizer(graph, initial).optimize();
}

void FactorGraphHandler::optimizeISAM2(uint8_t numberOfUpdates)
{
   LOG("optimizeISAM2()\n");
   isam.update(graph, initial);
   for (uint8_t i = 1; i < numberOfUpdates; i++)
   {
      isam.update();
   }
   result = isam.calculateEstimate();
   LOG("optimization complete()\n");
}

void FactorGraphHandler::clearISAM2()
{
   LOG("clearISAM2()\n");
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

void FactorGraphHandler::incrementPoseId()
{
   poseId++;
}

