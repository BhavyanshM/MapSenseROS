#include "FactorGraphHandler.h"

FactorGraphHandler::FactorGraphHandler()
{
   /* Set ISAM2 parameters here. */
   parameters.relinearizeThreshold = 0.01;
   parameters.relinearizeSkip = 1;
   this->isam = gtsam::ISAM2(parameters);

   gtsam::Vector6 odomVariance;
   odomVariance << 1e-2, 1e-2, 1e-2, 1e-1, 1e-1, 1e-1;
   createOdometryNoiseModel(odomVariance);

   gtsam::Vector3 lmVariance;
   lmVariance << 1e-2, 1e-2, 1e-2;
   createOrientedPlaneNoiseModel(lmVariance);

   gtsam::Vector6 priorVariance;
   priorVariance << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
   priorNoise = gtsam::noiseModel::Diagonal::Variances(priorVariance);

   gtsam::Vector6 priorVariance2;
   priorVariance2 << 1e2, 1e2, 1e2, 1e2, 1e2, 1e2;
   priorNoise2 = gtsam::noiseModel::Diagonal::Variances(priorVariance2);
}

void FactorGraphHandler::getPoses(std::vector<RigidBodyTransform>& poses)
{
   poses.clear();
   for (int i = 1; i < this->getPoseId(); i++)
   {
      RigidBodyTransform mapToSensorTransform(this->getResults().at<gtsam::Pose3>(gtsam::Symbol('x', i)).matrix());
      poses.emplace_back(mapToSensorTransform);
   }
}

void FactorGraphHandler::createOdometryNoiseModel(gtsam::Vector6 odomVariance)
{
   odometryNoise = gtsam::noiseModel::Diagonal::Variances(odomVariance);
}

void FactorGraphHandler::createOrientedPlaneNoiseModel(gtsam::Vector3 lmVariances)
{
   orientedPlaneNoise = gtsam::noiseModel::Diagonal::Variances(lmVariances);
}

int FactorGraphHandler::addPriorPoseFactor(gtsam::Pose3 mean)
{
   LOG("addPriorPoseFactor(x%d)\n", poseId);
   graph.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', poseId), mean, priorNoise));
   return poseId;
}

int FactorGraphHandler::addOdometryFactor(gtsam::Pose3 odometry)
{
   LOG("addOdometryFactor(x%d -o- x%d)\n", poseId, poseId + 1);
   graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol('x', poseId), gtsam::Symbol('x', poseId + 1), odometry, odometryNoise));
   poseId++;
   return poseId;
}

int FactorGraphHandler::addOrientedPlaneLandmarkFactor(gtsam::Vector4 lmMean, int lmId, int poseIndex)
{
   int landmarkId = (lmId != -1) ? lmId : newLandmarkId++;
   LOG("addOrientedPlaneLandmarkFactor(x%d -o- l%d)\n", poseIndex, landmarkId);
   graph.add(gtsam::OrientedPlane3Factor(lmMean, orientedPlaneNoise, gtsam::Symbol('x', poseIndex), gtsam::Symbol('l', landmarkId)));
   return landmarkId;
}

void FactorGraphHandler::setPoseInitialValue(int index, gtsam::Pose3 value)
{
   LOG("setPoseInitialValue(x%d)\n", index);
   if (structure.find('x' + std::to_string(index)) == structure.end())
   {
      structure.insert('x' + std::to_string(index));
      initial.insert(gtsam::Symbol('x', index), value);
   }
}

void FactorGraphHandler::setOrientedPlaneInitialValue(int index, gtsam::OrientedPlane3 value)
{
   LOG("setOrientedPlaneInitialValue(l%d)\n", index);
   if (!initial.exists(gtsam::Symbol('l', index)) && structure.find('l' + std::to_string(index)) == structure.end())
   {
      structure.insert('l' + std::to_string(index));
      initial.insert(gtsam::Symbol('l', index), value);
   }
}

void FactorGraphHandler::optimize()
{
   LOG("optimize()\n");
   result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
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

gtsam::Values FactorGraphHandler::getResults()
{
   return result;
}

gtsam::Values FactorGraphHandler::getInitial()
{
   return initial;
}

gtsam::NonlinearFactorGraph FactorGraphHandler::getFactorGraph()
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

