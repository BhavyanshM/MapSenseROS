#include "PlanarRegionMapHandler.h"

PlanarRegionMapHandler::PlanarRegionMapHandler()
{
   this->fgSLAM = new FactorGraphHandler();
}

void PlanarRegionMapHandler::registerRegionsPointToPlane(uint8_t iterations)
{
   Matrix4f T = Matrix4f::Identity();
   int totalNumOfBoundaryPoints = 0;
   for (int i = 0; i < this->matches.size(); i++)
   {
      totalNumOfBoundaryPoints += this->_latestRegionsZUp[this->matches[i].second]->GetNumOfBoundaryVertices();
   }
   MatrixXf A(totalNumOfBoundaryPoints, 6);
   VectorXf b(totalNumOfBoundaryPoints);

   int i = 0;
   for (int m = 0; m < this->matches.size(); m++)
   {
      for (int n = 0; n < this->_latestRegionsZUp[this->matches[m].second]->GetNumOfBoundaryVertices(); n++)
      {
         Vector3f latestPoint = _latestRegionsZUp[matches[m].second]->getVertices()[n];
         Vector3f correspondingMapCentroid = regions[matches[m].first]->GetCenter();
         Vector3f correspondingMapNormal = regions[matches[m].first]->GetNormal();
         Vector3f cross = latestPoint.cross(correspondingMapNormal);
         A(i, 0) = cross(0);
         A(i, 1) = cross(1);
         A(i, 2) = cross(2);
         A(i, 3) = correspondingMapNormal(0);
         A(i, 4) = correspondingMapNormal(1);
         A(i, 5) = correspondingMapNormal(2);
         b(i) = -(latestPoint - correspondingMapCentroid).dot(correspondingMapNormal);
         i++;
      }
   }

   //   printf("PlanarICP: (A:(%d, %d), b:(%d))\n", A.rows(), A.cols(), b.rows());

   VectorXf solution(6);
   solution = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);
   eulerAnglesToReference = Vector3d((double) solution(0), (double) solution(1), (double) solution(2));
   translationToReference = Vector3d((double) solution(3), (double) solution(4), (double) solution(5));

   //   printf("ICP Result: Rotation(%.2lf, %.2lf, %.2lf) Translation(%.2lf, %.2lf, %.2lf)\n", eulerAnglesToReference.x(), eulerAnglesToReference.y(),
   //          eulerAnglesToReference.z(), translationToReference.x(), translationToReference.y(), translationToReference.z());

   /* Update relative and total transform from current sensor pose to map frame. Required for initial value for landmarks observed in current pose. */
   _sensorPoseRelative.setRotationAndTranslation(eulerAnglesToReference, translationToReference);
   _sensorToMapTransform.multiplyRight(_sensorPoseRelative);
}

void PlanarRegionMapHandler::registerRegionsPointToPoint()
{
   Matrix4f T = Matrix4f::Identity();
   int totalNumOfBoundaryPoints = 0;
   for (int i = 0; i < this->matches.size(); i++)
   {
      totalNumOfBoundaryPoints += this->_latestRegionsZUp[this->matches[i].second]->GetNumOfBoundaryVertices();
   }
   MatrixXf A(totalNumOfBoundaryPoints, 6);
   VectorXf b(totalNumOfBoundaryPoints);

   int i = 0;
   for (int m = 0; m < this->matches.size(); m++)
   {
      for (int n = 0; n < this->_latestRegionsZUp[this->matches[m].second]->GetNumOfBoundaryVertices(); n++)
      {
         Vector3f latestPoint = _latestRegionsZUp[matches[m].second]->getVertices()[n];
         //         printf("(%d,%d,%d):(%.2lf,%.2lf,%.2lf)\n", m,n, i, latestPoint.x(), latestPoint.y(), latestPoint.z());
         Vector3f correspondingMapCentroid = regions[matches[m].first]->GetCenter();
         Vector3f correspondingMapNormal = regions[matches[m].first]->GetNormal();
         Vector3f cross = latestPoint.cross(correspondingMapNormal);
         A(i, 0) = cross(0);
         A(i, 1) = cross(1);
         A(i, 2) = cross(2);
         A(i, 3) = correspondingMapNormal(0);
         A(i, 4) = correspondingMapNormal(1);
         A(i, 5) = correspondingMapNormal(2);
         b(i) = -(latestPoint - correspondingMapCentroid).dot(correspondingMapNormal);
         i++;
      }
   }
   VectorXf solution(6);
   solution = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);
   eulerAnglesToReference = Vector3d((double) solution(0), (double) solution(1), (double) solution(2));
   translationToReference = Vector3d((double) solution(3), (double) solution(4), (double) solution(5));

   /* Update relative and total transform from current sensor pose to map frame. Required for initial value for landmarks observed in current pose. */
   _sensorPoseRelative = RigidBodyTransform(eulerAnglesToReference, translationToReference);
   _sensorToMapTransform.multiplyRight(_sensorPoseRelative);
}

void PlanarRegionMapHandler::matchPlanarRegionsToMap()
{
   matches.clear();
   for (int i = 0; i < regions.size(); i++)
   {
      if (regions[i]->GetNumOfBoundaryVertices() > 8)
      {
         for (int j = 0; j < _latestRegionsZUp.size(); j++)
         {
            if (_latestRegionsZUp[j]->GetNumOfBoundaryVertices() > 8)
            {
               Vector3f prevNormal = regions[i]->GetNormal();
               Vector3f curNormal = _latestRegionsZUp[j]->GetNormal();
               float angularDiff = fabs(prevNormal.dot(curNormal));

               Vector3f prevCenter = regions[i]->GetCenter();
               Vector3f curCenter = _latestRegionsZUp[j]->GetCenter();
               float dist = (curCenter - prevCenter).norm();
               //         float dist = fabs((prevCenter - curCenter).dot(curNormal)) + fabs((curCenter - prevCenter).dot(prevNormal));

               int countDiff = abs(regions[i]->GetNumOfBoundaryVertices() - _latestRegionsZUp[j]->GetNumOfBoundaryVertices());
               int maxCount = max(regions[i]->GetNumOfBoundaryVertices(), _latestRegionsZUp[j]->GetNumOfBoundaryVertices());

               if (dist < MATCH_DIST_THRESHOLD && angularDiff > MATCH_ANGULAR_THRESHOLD &&
                   ((float) countDiff / ((float) maxCount)) * 100.0f < MATCH_PERCENT_VERTEX_THRESHOLD)
               {
                  matches.emplace_back(i, j);
                  _latestRegionsZUp[j]->setId(regions[i]->getId());
                  regions[i]->SetNumOfMeasurements(regions[i]->GetNumOfMeasurements() + 1);
                  _latestRegionsZUp[j]->SetNumOfMeasurements(_latestRegionsZUp[j]->GetNumOfMeasurements() + 1);
                  break;
               }
            }
         }
      }
   }
}

void PlanarRegionMapHandler::insertOrientedPlaneFactors(int currentPoseId)
{
   for (int i = 0; i < _latestRegionsZUp.size(); i++)
   {
      shared_ptr<PlanarRegion> region = _latestRegionsZUp[i];
      Eigen::Vector4d plane;
      plane << region->GetNormal().cast<double>(), (double) -region->GetNormal().dot(region->GetCenter());
      region->setId(fgSLAM->addOrientedPlaneLandmarkFactor(plane, region->getId(), currentPoseId));
      region->setPoseId(currentPoseId);
   }
}

void PlanarRegionMapHandler::setOrientedPlaneInitialValues()
{
   for (auto region : regionsInMapFrame)
   {
      Eigen::Vector4d plane;
      plane << region->GetNormal().cast<double>(), (double) -region->GetNormal().dot(region->GetCenter());
      this->fgSLAM->setOrientedPlaneInitialValue(region->getId(), gtsam::OrientedPlane3(plane(0), plane(1), plane(2), plane(3)));
   }
}

void PlanarRegionMapHandler::mergeLatestRegions()
{
   for (shared_ptr<PlanarRegion> region : this->_latestRegionsZUp)
   {
      if (region->GetNumOfMeasurements() < 2 && region->GetPoseId() != 0)
      {
         this->measuredRegions.emplace_back(region);
      }
   }
}

void PlanarRegionMapHandler::extractFactorGraphLandmarks()
{
   mapRegions.clear();
   for (shared_ptr<PlanarRegion> region : this->_latestRegionsZUp)
   {
      RigidBodyTransform mapToSensorTransform(fgSLAM->getResults().at<gtsam::Pose3>(gtsam::Symbol('x', region->GetPoseId())).matrix());

      shared_ptr<PlanarRegion> transformedRegion = std::make_shared<PlanarRegion>(region->getId());
      region->CopyAndTransform(transformedRegion, mapToSensorTransform);

      transformedRegion->ProjectToPlane(fgSLAM->getResults().at<gtsam::OrientedPlane3>(gtsam::Symbol('l', region->getId())).planeCoefficients().cast<float>());
      mapRegions.emplace_back(transformedRegion);
   }
}

void PlanarRegionMapHandler::optimize()
{
   this->ISAM2 ? this->fgSLAM->optimizeISAM2(this->ISAM2_NUM_STEPS) : this->fgSLAM->optimize();
}

void PlanarRegionMapHandler::setDirectory(const string& directory)
{
   this->directory = directory;
}

void PlanarRegionMapHandler::transformAndCopyRegions(const vector<shared_ptr<PlanarRegion>>& srcRegions, vector<shared_ptr<PlanarRegion>>& dstRegions, const RigidBodyTransform& transform)
{
   dstRegions.clear();
   for (int i = 0; i < srcRegions.size(); i++)
   {
      shared_ptr<PlanarRegion> planarRegion = std::make_shared<PlanarRegion>(srcRegions[i]->getId());
      srcRegions[i]->CopyAndTransform(planarRegion, transform);
      dstRegions.emplace_back(planarRegion);
   }
}

void PlanarRegionMapHandler::printRefCounts()
{
   printf("---------- REF Counts ----------\n");
   printf("Regions(");
   for (auto region : this->regions)
      printf("%d, ", region.use_count());
   printf(")\n");
   printf("LatestRegions(");
   for (auto region : this->_latestRegionsZUp)
      printf("%d, ", region.use_count());
   printf(")\n");
   printf("MapRegions(");
   for (auto region : this->mapRegions)
      printf("%d, ", region.use_count());
   printf(")\n");
   printf("RegionsInMapFrame(");
   for (auto region : this->regionsInMapFrame)
      printf("%d, ", region.use_count());
   printf(")\n");
   printf("---------- REF Counts End ----------\n\n");
}








