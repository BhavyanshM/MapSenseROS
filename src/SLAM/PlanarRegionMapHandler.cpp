#include "PlanarRegionMapHandler.h"

void PlanarRegionMapHandler::registerRegionsPointToPlane()
{
   Matrix4f T = Matrix4f::Identity();
   int totalNumOfBoundaryPoints = 0;
   for (int i = 0; i < this->matches.size(); i++)
   {
      totalNumOfBoundaryPoints += this->latestRegions[this->matches[i].second]->getNumOfBoundaryVertices();
   }
   MatrixXf A(totalNumOfBoundaryPoints, 6);
   VectorXf b(totalNumOfBoundaryPoints);

   int i = 0;
   for (int m = 0; m < this->matches.size(); m++)
   {
      for (int n = 0; n < this->latestRegions[this->matches[m].second]->getNumOfBoundaryVertices(); n++)
      {
         Vector3f latestPoint = latestRegions[matches[m].second]->getVertices()[n];
         //         printf("(%d,%d,%d):(%.2lf,%.2lf,%.2lf)\n", m,n, i, latestPoint.x(), latestPoint.y(), latestPoint.z());
         Vector3f correspondingMapCentroid = regions[matches[m].first]->getCenter();
         Vector3f correspondingMapNormal = regions[matches[m].first]->getNormal();
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
   _sensorToMapTransform.appendRight(_sensorPoseRelative);
}

void PlanarRegionMapHandler::registerRegionsPointToPoint()
{
   Matrix4f T = Matrix4f::Identity();
   int totalNumOfBoundaryPoints = 0;
   for (int i = 0; i < this->matches.size(); i++)
   {
      totalNumOfBoundaryPoints += this->latestRegions[this->matches[i].second]->getNumOfBoundaryVertices();
   }
   MatrixXf A(totalNumOfBoundaryPoints, 6);
   VectorXf b(totalNumOfBoundaryPoints);

   int i = 0;
   for (int m = 0; m < this->matches.size(); m++)
   {
      for (int n = 0; n < this->latestRegions[this->matches[m].second]->getNumOfBoundaryVertices(); n++)
      {
         Vector3f latestPoint = latestRegions[matches[m].second]->getVertices()[n];
         //         printf("(%d,%d,%d):(%.2lf,%.2lf,%.2lf)\n", m,n, i, latestPoint.x(), latestPoint.y(), latestPoint.z());
         Vector3f correspondingMapCentroid = regions[matches[m].first]->getCenter();
         Vector3f correspondingMapNormal = regions[matches[m].first]->getNormal();
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
   _sensorToMapTransform.appendRight(_sensorPoseRelative);
}

void PlanarRegionMapHandler::matchPlanarRegionsToMap(vector<shared_ptr<PlanarRegion>> latestRegions)
{
   matches.clear();
   for (int i = 0; i < regions.size(); i++)
   {
      if (regions[i]->getNumOfBoundaryVertices() > 8)
      {
         for (int j = 0; j < latestRegions.size(); j++)
         {
            if (latestRegions[j]->getNumOfBoundaryVertices() > 8)
            {
               Vector3f prevNormal = regions[i]->getNormal();
               Vector3f curNormal = latestRegions[j]->getNormal();
               float angularDiff = fabs(prevNormal.dot(curNormal));

               Vector3f prevCenter = regions[i]->getCenter();
               Vector3f curCenter = latestRegions[j]->getCenter();
               float dist = (curCenter - prevCenter).norm();
               //         float dist = fabs((prevCenter - curCenter).dot(curNormal)) + fabs((curCenter - prevCenter).dot(prevNormal));

               int countDiff = abs(regions[i]->getNumOfBoundaryVertices() - latestRegions[j]->getNumOfBoundaryVertices());
               int maxCount = max(regions[i]->getNumOfBoundaryVertices(), latestRegions[j]->getNumOfBoundaryVertices());

               if (dist < MATCH_DIST_THRESHOLD && angularDiff > MATCH_ANGULAR_THRESHOLD &&
                   ((float) countDiff / ((float) maxCount)) * 100.0f < MATCH_PERCENT_VERTEX_THRESHOLD)
               {
                  matches.emplace_back(i, j);
                  latestRegions[j]->setId(regions[i]->getId());
                  regions[i]->setNumOfMeasurements(regions[i]->getNumOfMeasurements() + 1);
                  latestRegions[j]->setNumOfMeasurements(latestRegions[j]->getNumOfMeasurements() + 1);
                  break;
               }
            }
         }
      }
   }
}



Vector3f getVec3f(string csv)
{
   vector<string> CSVSubStrings;
   stringstream csvStream(csv);
   string csvStr;
   while (getline(csvStream, csvStr, ','))
   {
      CSVSubStrings.push_back(csvStr);
   }
   //   cout << "Vector:" << Vector3f(stof(CSVSubStrings[0]), stof(CSVSubStrings[1]), stof(CSVSubStrings[2])) << endl;
   return Vector3f(stof(CSVSubStrings[0]), stof(CSVSubStrings[1]), stof(CSVSubStrings[2]));
}

void getNextLineSplit(ifstream& regionFile, vector<string>& subStrings)
{
   subStrings.clear();
   string regionText;
   getline(regionFile, regionText);
   stringstream ss(regionText);
   string str;
   while (getline(ss, str, ':'))
   {
      //      cout << str << '\t';
      subStrings.push_back(str);
   }
   //   cout << endl;
}

void PlanarRegionMapHandler::loadRegions(int frameId, vector<shared_ptr<PlanarRegion>>& regions)
{
   /* Generate planar region objects from the sorted list of files. */
   regions.clear();
   ifstream regionFile(directory + files[frameId]);
   cout << "Loading Regions From: " << directory + files[frameId] << endl;
   vector<string> subStrings;
   getNextLineSplit(regionFile, subStrings); // Get number of regions
   int numRegions = stoi(subStrings[1]);
   for (int r = 0; r < numRegions; r++) // For each region
   {
      shared_ptr<PlanarRegion> region = std::make_shared<PlanarRegion>(0);
      getNextLineSplit(regionFile, subStrings); // Get regionId
      region->setId(-1);
      //      region->setId(stoi(subStrings[1]));
      getNextLineSplit(regionFile, subStrings); // Get regionCenter
      region->setCenter(getVec3f(subStrings[1]));
      getNextLineSplit(regionFile, subStrings); // Get regionNormal
      region->setNormal(getVec3f(subStrings[1]));
      getNextLineSplit(regionFile, subStrings); // Get numBoundaryVertices
      int length = stoi(subStrings[1]);
      for (int i = 0; i < length; i++)
      {
         getNextLineSplit(regionFile, subStrings);
         region->insertBoundaryVertex(getVec3f(subStrings[0]));
      }
//      GeomTools::compressPointSetLinear(region);
      regions.emplace_back(region);
   }
}

void PlanarRegionMapHandler::transformLatestRegions(RigidBodyTransform transform)
{
   for (int i = 0; i < this->latestRegions.size(); i++)
   {
      this->latestRegions[i]->transform(transform);
   }
}

void PlanarRegionMapHandler::transformLatestRegions(Vector3d translation, Matrix3d rotation)
{
   for (int i = 0; i < this->latestRegions.size(); i++)
   {
      this->latestRegions[i]->transform(translation, rotation);
   }
}

void PlanarRegionMapHandler::transformAndCopyLatestRegions(RigidBodyTransform transform, vector<shared_ptr<PlanarRegion>>& transformedRegions)
{
   for (int i = 0; i < latestRegions.size(); i++)
   {
      shared_ptr<PlanarRegion> planarRegion = std::make_shared<PlanarRegion>(latestRegions[i]->getId());
      latestRegions[i]->copyAndTransform(planarRegion, transform);
      transformedRegions.emplace_back(planarRegion);
   }
}

void PlanarRegionMapHandler::updateFactorGraphLandmarks(vector<shared_ptr<PlanarRegion>>& regionsToInsert, int currentPoseId)
{
   for (int i = 0; i < regionsToInsert.size(); i++)
   {
      shared_ptr<PlanarRegion> region = regionsToInsert[i];
      Eigen::Vector4d plane;
      plane << region->getNormal().cast<double>(), (double) -region->getNormal().dot(region->getCenter());
      region->setId(fgSLAM.addOrientedPlaneLandmarkFactor(plane, region->getId()));
      region->setPoseId(currentPoseId);
   }
}

int PlanarRegionMapHandler::updateFactorGraphPoses(RigidBodyTransform odometry)
{
   return fgSLAM.addOdometryFactor(MatrixXd(odometry.getMatrix()));
}

void PlanarRegionMapHandler::initFactorGraphState(RigidBodyTransform sensorPose, vector<shared_ptr<PlanarRegion>> regionsInMapFrame)
{
   this->fgSLAM.initPoseValue(MatrixXd(sensorPose.getMatrix()));
   for (auto region : regionsInMapFrame)
   {
      Eigen::Vector4d plane;
      plane << region->getNormal().cast<double>(), (double) -region->getNormal().dot(region->getCenter());
      this->fgSLAM.initOrientedPlaneLandmarkValue(region->getId(), plane);
   }
}

void PlanarRegionMapHandler::mergeLatestRegions()
{
   for (shared_ptr<PlanarRegion> region : this->latestRegions)
   {
      if (region->getNumOfMeasurements() < 2 && region->getPoseId() != 0)
      {
         this->measuredRegions.emplace_back(region);
      }
   }
}

void PlanarRegionMapHandler::updateMapRegionsWithSLAM()
{
   mapRegions.clear();
   for (shared_ptr<PlanarRegion> region : this->latestRegions)
   {
      RigidBodyTransform mapToSensorTransform(fgSLAM.getResults().at<Pose3>(Symbol('x', region->getPoseId())).matrix());

      shared_ptr<PlanarRegion> transformedRegion = std::make_shared<PlanarRegion>(region->getId());
      region->copyAndTransform(transformedRegion, mapToSensorTransform);

      transformedRegion->projectToPlane(fgSLAM.getResults().at<OrientedPlane3>(Symbol('l', region->getId())).planeCoefficients().cast<float>());
      mapRegions.emplace_back(transformedRegion);
   }
   cout << "Total Map Regions: " << mapRegions.size() << endl;
}

void PlanarRegionMapHandler::setDirectory(const string& directory)
{
   this->directory = directory;
}








