#include "MapHandler.h"
#include "ImGuiTools.h"

MapHandler::MapHandler()
{
//   this->fgSLAM = new FactorGraphHandler();
   AppUtils::getFileNames("/home/quantum/Workspace/Volume/catkin_ws/src/MapSenseROS/Extras/Regions/Archive/Set_06_Circle/", fileNames);

   _transformZUp.rotateZ(-90.0f / 180.0f * M_PI);
   _transformZUp.rotateY(90.0f / 180.0f * M_PI);
}

void MapHandler::ImGuiUpdate()
{
   if(ImGui::BeginTabItem("Mapper"))
   {

      if(ImGui::BeginTabBar("Map"))
      {
         if(ImGui::BeginTabItem("Plotter"))
         {
            ImGuiTools::GetDropDownSelection("File", fileNames, fileSelected);
            if(ImGui::Button("Load Regions"))
            {
               _regionCalculator->LoadRegions("/home/quantum/Workspace/Volume/catkin_ws/src/MapSenseROS/Extras/Regions/Archive/Set_06_Circle/", fileNames, fileSelected);
            }
            std::vector<std::shared_ptr<PlanarRegion>> regions = _regionCalculator->planarRegionList;
            ImGui::Text("Total Planar Regions: %d", regions.size());

            ImGuiTools::GetDropDownSelection("Region", "Region", regionSelected, regions.size());
            //      ImGui::SameLine(ImGui::GetWindowWidth() - 30);
            if(ImGui::Button("Plot"))
            {
               plotter2D = true;
            }


            ImGui::SliderFloat("Segment Dist Threshold", &SEGMENT_DIST_THRESHOLD, 0.01f, 0.5f);
            ImGui::SliderFloat("Compress Dist Threshold", &COMPRESS_DIST_THRESHOLD, 0.01f, 0.1f);
            ImGui::SliderFloat("Compress Cosine Threshold", &COMPRESS_COSINE_THRESHOLD, 0.01f, 1.0f);

            if(plotter2D && regions.size() > 0)
            {

               regions[regionSelected]->ComputeBoundaryVerticesPlanar();
               regions[regionSelected]->ComputeSegmentIndices(SEGMENT_DIST_THRESHOLD);
               regions[regionSelected]->CompressRegionSegmentsLinear(COMPRESS_DIST_THRESHOLD, COMPRESS_COSINE_THRESHOLD);

               ImGuiTools::GetDropDownSelection("Segment", "Segment", segmentSelected, regions.size());
               const std::vector<Eigen::Vector2f>& points = regions[regionSelected]->GetPlanarPatchCentroids();
               const std::vector<int>& segmentIndices = regions[regionSelected]->GetSegmentIndices();
               Eigen::Vector2f mousePlotLocation = ImGuiTools::ScatterPlotRegionSegments(points, segmentIndices);

               ImGui::Text("Mouse Plot: %.2lf, %.2lf", mousePlotLocation.x(), mousePlotLocation.y());
               ImGui::Text("Winding Number: %.2lf", GeomTools::ComputeWindingNumber(regions[regionSelected]->GetPlanarPatchCentroids(), mousePlotLocation));

               if(ImGui::Button("Close"))
               {
                  plotter2D = false;
               }
            }
            ImGui::EndTabItem();
         }

         if(ImGui::BeginTabItem("Map"))
         {
            if(ImGui::Button("Register"))
            {
               _regionCalculator->LoadRegions("/home/quantum/Workspace/Volume/catkin_ws/src/MapSenseROS/Extras/Regions/Archive/Set_06_Circle/", fileNames, _frameIndex );
               Update(_regionCalculator->planarRegionList);
            }
            ImGui::EndTabItem();
         }

         ImGui::EndTabBar();
      }



      ImGui::EndTabItem();
   }


}

void MapHandler::Update(std::vector <std::shared_ptr<PlanarRegion>>& regions)
{

   if(_mapRegions.size() == 0)
   {
      TransformAndCopyRegions(latestRegions, _mapRegions, _transformZUp);
   }

   latestRegions = std::move(regions);
   TransformAndCopyRegions(latestRegions, _latestRegionsZUp, _transformZUp);

   MatchPlanarRegionsToMap();
   ROS_DEBUG("Regions Matched: (%d).\n", _matches.size());

   if (_matches.size() > 0)
      RegisterRegionsPointToPlane(1);

   int currentPoseId = 1;
}

void MapHandler::RegisterRegionsPointToPlane(uint8_t iterations)
{
   Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
   int totalNumOfBoundaryPoints = 0;
   for (int i = 0; i < _matches.size(); i++)
   {
      totalNumOfBoundaryPoints += _latestRegionsZUp[_matches[i].second]->GetNumOfBoundaryVertices();
   }
   Eigen::MatrixXf A(totalNumOfBoundaryPoints, 6);
   Eigen::VectorXf b(totalNumOfBoundaryPoints);

   int i = 0;
   for (int m = 0; m < _matches.size(); m++)
   {
      for (int n = 0; n < _latestRegionsZUp[_matches[m].second]->GetNumOfBoundaryVertices(); n++)
      {
         Eigen::Vector3f latestPoint = _latestRegionsZUp[_matches[m].second]->getVertices()[n];
         Eigen::Vector3f correspondingMapCentroid = regions[_matches[m].first]->GetCenter();
         Eigen::Vector3f correspondingMapNormal = regions[_matches[m].first]->GetNormal();
         Eigen::Vector3f cross = latestPoint.cross(correspondingMapNormal);
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

   printf("PlanarICP: (A:(%d, %d), b:(%d))\n", A.rows(), A.cols(), b.rows());

   Eigen::VectorXf solution(6);
   solution = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
   eulerAnglesToReference = Eigen::Vector3d((double) solution(0), (double) solution(1), (double) solution(2));
   translationToReference = Eigen::Vector3d((double) solution(3), (double) solution(4), (double) solution(5));

   printf("ICP Result: Rotation(%.2lf, %.2lf, %.2lf) Translation(%.2lf, %.2lf, %.2lf)\n", eulerAnglesToReference.x(), eulerAnglesToReference.y(),
          eulerAnglesToReference.z(), translationToReference.x(), translationToReference.y(), translationToReference.z());

   /* Update relative and total transform from current sensor pose to map frame. Required for initial value for landmarks observed in current pose. */
   _sensorPoseRelative.setRotationAndTranslation(eulerAnglesToReference, translationToReference);
   _sensorToMapTransform.multiplyRight(_sensorPoseRelative);
}

void MapHandler::RegisterRegionsPointToPoint()
{
   Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
   int totalNumOfBoundaryPoints = 0;
   for (int i = 0; i < _matches.size(); i++)
   {
      totalNumOfBoundaryPoints += _latestRegionsZUp[_matches[i].second]->GetNumOfBoundaryVertices();
   }
   Eigen::MatrixXf A(totalNumOfBoundaryPoints, 6);
   Eigen::VectorXf b(totalNumOfBoundaryPoints);

   int i = 0;
   for (int m = 0; m < _matches.size(); m++)
   {
      for (int n = 0; n < _latestRegionsZUp[_matches[m].second]->GetNumOfBoundaryVertices(); n++)
      {
         Eigen::Vector3f latestPoint = _latestRegionsZUp[_matches[m].second]->getVertices()[n];
         //         printf("(%d,%d,%d):(%.2lf,%.2lf,%.2lf)\n", m,n, i, latestPoint.x(), latestPoint.y(), latestPoint.z());
         Eigen::Vector3f correspondingMapCentroid = regions[_matches[m].first]->GetCenter();
         Eigen::Vector3f correspondingMapNormal = regions[_matches[m].first]->GetNormal();
         Eigen::Vector3f cross = latestPoint.cross(correspondingMapNormal);
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
   Eigen::VectorXf solution(6);
   solution = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
   eulerAnglesToReference = Eigen::Vector3d((double) solution(0), (double) solution(1), (double) solution(2));
   translationToReference = Eigen::Vector3d((double) solution(3), (double) solution(4), (double) solution(5));

   /* Update relative and total transform from current sensor pose to map frame. Required for initial value for landmarks observed in current pose. */
   _sensorPoseRelative = RigidBodyTransform(eulerAnglesToReference, translationToReference);
   _sensorToMapTransform.multiplyRight(_sensorPoseRelative);
}

void MapHandler::MatchPlanarRegionsToMap()
{
   _matches.clear();
   for (int i = 0; i < regions.size(); i++)
   {
      if (regions[i]->GetNumOfBoundaryVertices() > 8)
      {
         for (int j = 0; j < _latestRegionsZUp.size(); j++)
         {
            if (_latestRegionsZUp[j]->GetNumOfBoundaryVertices() > 8)
            {
               Eigen::Vector3f prevNormal = regions[i]->GetNormal();
               Eigen::Vector3f curNormal = _latestRegionsZUp[j]->GetNormal();
               float angularDiff = fabs(prevNormal.dot(curNormal));

               Eigen::Vector3f prevCenter = regions[i]->GetCenter();
               Eigen::Vector3f curCenter = _latestRegionsZUp[j]->GetCenter();
               float dist = (curCenter - prevCenter).norm();
               //         float dist = fabs((prevCenter - curCenter).dot(curNormal)) + fabs((curCenter - prevCenter).dot(prevNormal));

               int countDiff = abs(regions[i]->GetNumOfBoundaryVertices() - _latestRegionsZUp[j]->GetNumOfBoundaryVertices());
               int maxCount = std::max(regions[i]->GetNumOfBoundaryVertices(), _latestRegionsZUp[j]->GetNumOfBoundaryVertices());

               if (dist < MATCH_DIST_THRESHOLD && angularDiff > MATCH_ANGULAR_THRESHOLD &&
                   ((float) countDiff / ((float) maxCount)) * 100.0f < MATCH_PERCENT_VERTEX_THRESHOLD)
               {
                  _matches.emplace_back(i, j);
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

void MapHandler::InsertMapRegions(const std::vector<std::shared_ptr<PlanarRegion>>& regions)
{
   for (auto region : regions)
   {
      _mapRegions.push_back(std::move(region));
   }
}

//void MapHandler::InsertOrientedPlaneFactors(int currentPoseId)
//{
//   for (int i = 0; i < _latestRegionsZUp.size(); i++)
//   {
//      shared_ptr<PlanarRegion> region = _latestRegionsZUp[i];
//      Eigen::Vector4d plane;
//      plane << region->GetNormal().cast<double>(), (double) -region->GetNormal().dot(region->GetCenter());
//      region->setId(fgSLAM->addOrientedPlaneLandmarkFactor(plane, region->getId(), currentPoseId));
//      region->setPoseId(currentPoseId);
//   }
//}
//
//void MapHandler::SetOrientedPlaneInitialValues()
//{
//   for (auto region : regionsInMapFrame)
//   {
//      Eigen::Vector4d plane;
//      plane << region->GetNormal().cast<double>(), (double) -region->GetNormal().dot(region->GetCenter());
//      this->fgSLAM->setOrientedPlaneInitialValue(region->getId(), gtsam::OrientedPlane3(plane(0), plane(1), plane(2), plane(3)));
//   }
//}
//
//void MapHandler::MergeLatestRegions()
//{
//   for (shared_ptr<PlanarRegion> region : this->_latestRegionsZUp)
//   {
//      if (region->GetNumOfMeasurements() < 2 && region->GetPoseId() != 0)
//      {
//         this->measuredRegions.emplace_back(region);
//      }
//   }
//}
//
//void MapHandler::ExtractFactorGraphLandmarks()
//{
//   _mapRegions.clear();
//   for (shared_ptr<PlanarRegion> region : this->_latestRegionsZUp)
//   {
//      RigidBodyTransform mapToSensorTransform(fgSLAM->getResults().at<gtsam::Pose3>(gtsam::Symbol('x', region->GetPoseId())).matrix());
//
//      shared_ptr<PlanarRegion> transformedRegion = std::make_shared<PlanarRegion>(region->getId());
//      region->CopyAndTransform(transformedRegion, mapToSensorTransform);
//
//      transformedRegion->ProjectToPlane(fgSLAM->getResults().at<gtsam::OrientedPlane3>(gtsam::Symbol('l', region->getId())).planeCoefficients().cast<float>());
//      _mapRegions.emplace_back(transformedRegion);
//   }
//}
//
//void MapHandler::Optimize()
//{
//   this->ISAM2 ? this->fgSLAM->optimizeISAM2(this->ISAM2_NUM_STEPS) : this->fgSLAM->optimize();
//}

void MapHandler::setDirectory(const std::string& directory)
{
   this->directory = directory;
}

void MapHandler::TransformAndCopyRegions(const std::vector<std::shared_ptr<PlanarRegion>>& srcRegions, std::vector<std::shared_ptr<PlanarRegion>>& dstRegions, const RigidBodyTransform& transform)
{
   dstRegions.clear();
   for (int i = 0; i < srcRegions.size(); i++)
   {
      std::shared_ptr<PlanarRegion> planarRegion = std::make_shared<PlanarRegion>(srcRegions[i]->getId());
      srcRegions[i]->CopyAndTransform(planarRegion, transform);
      dstRegions.emplace_back(planarRegion);
   }
}

void MapHandler::PrintRefCounts()
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
   for (auto region : this->_mapRegions)
      printf("%d, ", region.use_count());
   printf(")\n");
   printf("RegionsInMapFrame(");
   for (auto region : this->regionsInMapFrame)
      printf("%d, ", region.use_count());
   printf(")\n");
   printf("---------- REF Counts End ----------\n\n");
}








