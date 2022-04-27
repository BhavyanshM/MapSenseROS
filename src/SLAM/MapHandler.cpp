#include "MapHandler.h"
#include "ImGuiTools.h"
#include "PoseTrajectory.h"

MapHandler::MapHandler(NetworkManager *network, ApplicationState& app) : _app(app),
                                                                         _fileBrowserUI(ros::package::getPath("map_sense") + "/Extras/Regions/Archive")
{
   _network = network;

   _directory = ros::package::getPath("map_sense") + "/Extras/Regions/Archive/Set_Ouster_Convex_ISR_01/";
   AppUtils::getFileNames(_directory, fileNames);

   /* TODO: Temporary. */
   GeomTools::LoadRegions(0, _previousRegionsZUp, _directory, fileNames);
   GeomTools::LoadRegions(1, _latestRegionsZUp, _directory, fileNames);

   //   _transformZUp.RotateZ(-90.0f / 180.0f * M_PI);
   //   _transformZUp.RotateY(90.0f / 180.0f * M_PI);
}

void MapHandler::ImGuiUpdate(ApplicationState& app)
{
   _app = app;

   if (ImGui::BeginTabItem("Mapper"))
   {
      if (ImGui::BeginTabBar("Mapper Tabs"))
      {
         if (ImGui::BeginTabItem("Trajectory"))
         {

            if (ImGui::Button("Plot Trajectory"))
            {
               plotter2D = true;
            }

            ImGui::SliderFloat("EndX", &endX, -10, 10);
            ImGui::SliderFloat("EndY", &endY, -10, 10);
            ImGui::SliderFloat("EndZ", &endZ, -10, 10);
            ImGui::SliderFloat("EndRoll", &endRoll, -10, 10);
            ImGui::SliderFloat("EndPitch", &endPitch, -10, 10);
            ImGui::SliderFloat("EndYaw", &endYaw, -10, 10);
            ImGui::SliderFloat("VelEndX", &velEndX, -10, 10);
            ImGui::SliderFloat("VelEndY", &velEndY, -10, 10);
            ImGui::SliderFloat("VelEndZ", &velEndZ, -10, 10);

//            if (plotter2D)
//            {
//               PoseTrajectory pose;
//
//               pose.SetPitchConditions(0, 1, 0, endPitch, 0, 0);
//               pose.SetRollConditions(0, 1, 0, endRoll, 0, 0);
//               pose.SetYawConditions(0, 1, 0, endYaw, 0, 0);
//               pose.SetXConditions(0, 1, 0, endX, 0, velEndX);
//               pose.SetYConditions(0, 1, 0, endY, 0, velEndY);
//               pose.SetZConditions(0, 1, 0, endZ, 0, velEndZ);
//               pose.Optimize();
//
//               auto position = pose.GetPosition(0.5);
//
//               int totalSamples = 80;
//               float timeUnit = 1.0f / (float) totalSamples;
//               std::vector<Eigen::Vector2f> points;
//               _mesher->ClearPoses();
//               for (int i = 0; i < totalSamples + 1; i++)
//               {
//                  _mesher->GeneratePoseMesh(pose.GetPose(timeUnit * (float) i).GetMatrix().cast<float>(), nullptr);
//               }
//            }
            ImGui::EndTabItem();
         }

         if (ImGui::BeginTabItem("Plot"))
         {
            ImGuiTools::GetDropDownSelection("File", fileNames, fileSelected);
            if (ImGui::Button("Load Regions"))
            {
               GeomTools::LoadRegions(fileSelected, latestRegions, _directory, fileNames);
            }

            ImGui::Text("File Chosen: %s", _directory.c_str());
            if (ImGui::Button("Browse"))
            {
               showFileBrowser = true;
            }
            if (showFileBrowser)
            {
               showFileBrowser = _fileBrowserUI.ImGuiUpdate(_directory);
               if (!showFileBrowser && _directory.substr(_directory.length() - 4, 4).compare(".txt") == 0)
                  GeomTools::LoadRegions(_directory, _latestRegionsZUp, true);
               _mesher->GenerateMeshForRegions(_latestRegionsZUp, nullptr, true);
            }

            ImGui::Text("Total Planar Regions: %d", latestRegions.size());

            ImGuiTools::GetDropDownSelection("Region", "Region", regionSelected, latestRegions.size());
            ImGui::SameLine(ImGui::GetWindowWidth() - 40);
            if (ImGui::Button("Plot"))
            {
               plotter2D = true;
            }

            ImGui::SliderFloat("Segment Dist Threshold", &app.SEGMENT_DIST_THRESHOLD, 0.01f, 0.5f);
            ImGui::SliderFloat("Compress Dist Threshold", &app.COMPRESS_DIST_THRESHOLD, 0.01f, 0.1f);
            ImGui::SliderFloat("Compress Cosine Threshold", &app.COMPRESS_COSINE_THRESHOLD, 0.01f, 1.0f);

//            if (plotter2D && latestRegions.size() > 0)
//            {
//
//               latestRegions[regionSelected]->ComputeBoundaryVerticesPlanar();
//               const std::vector<Eigen::Vector2f>& boundaryPoints2D = latestRegions[regionSelected]->GetPlanarPatchCentroids();
//
//               latestRegions[regionSelected + 1]->ComputeBoundaryVerticesPlanar();
//               const std::vector<Eigen::Vector2f>& boundaryPointsSecond2D = latestRegions[regionSelected + 1]->GetPlanarPatchCentroids();
//
//               latestRegions[regionSelected + 2]->ComputeBoundaryVerticesPlanar();
//               const std::vector<Eigen::Vector2f>& boundaryPointsThird2D = latestRegions[regionSelected + 2]->GetPlanarPatchCentroids();
//
//               auto unionHull = HullTools::CalculateUnion(boundaryPoints2D, boundaryPointsSecond2D);
//               auto combinedUnionHull = HullTools::CalculateUnion(unionHull, boundaryPointsThird2D);
//
//               float IoU = HullTools::ComputeBoundingBoxIoU({boundaryPoints2D}, {boundaryPointsSecond2D});
//
//               ImGui::Text("IoU for Hulls: %.2lf", IoU);
//
//               Eigen::Vector2f mousePlotLocation;
//
//               if (ImGuiTools::BeginPlotWindow("Plotter 2D"))
//               {
//                  ImGuiTools::ScatterPlotXY(boundaryPoints2D, "Region 1", true);
//                  ImGuiTools::ScatterPlotXY(boundaryPointsSecond2D, "Region 2", true);
//                  ImGuiTools::ScatterPlotXY(boundaryPointsThird2D, "Region 3", true);
//
//                  ImGuiTools::ScatterPlotXY(unionHull, "Union Hull");
//                  ImGuiTools::ScatterPlotXY(combinedUnionHull, "Combined Union Hull");
//
//                  ImPlotPoint mouse = ImPlot::GetPlotMousePos();
//                  mousePlotLocation << mouse.x, mouse.y;
//
//                  ImGuiTools::EndPlotWindow();
//               }
//
//               ImGui::Text("Mouse Plot: %.2lf, %.2lf", mousePlotLocation.x(), mousePlotLocation.y());
//               ImGui::Text("Winding Number Region 1: %.2lf", HullTools::ComputeWindingNumber(boundaryPoints2D, mousePlotLocation));
//               ImGui::Text("Winding Number Region 2: %.2lf", HullTools::ComputeWindingNumber(boundaryPointsSecond2D, mousePlotLocation));
//               ImGui::Text("Winding Number Union Hull: %.2lf", HullTools::ComputeWindingNumber(unionHull, mousePlotLocation));
//
//               //               latestRegions[regionSelected]->ComputeSegmentIndices(app.SEGMENT_DIST_THRESHOLD);
//               //               latestRegions[regionSelected]->CompressRegionSegmentsLinear(app.COMPRESS_DIST_THRESHOLD, app.COMPRESS_COSINE_THRESHOLD);
//
//               //               ImGuiTools::GetDropDownSelection("Segment", "Segment", segmentSelected, latestRegions.size());
//               //               const std::vector<int>& segmentIndices = latestRegions[regionSelected]->GetSegmentIndices();
//
//
//
//
//
//
//               if (ImGui::Button("Close"))
//               {
//                  plotter2D = false;
//               }
//            }
            ImGui::EndTabItem();
         }

         if (ImGui::BeginTabItem("Map"))
         {
            ImGui::Text("Frame Index: %d", _frameIndex);
            ImGui::Text("Rapid Regions Loaded: %d", _regionCalculator->planarRegionList.size());
            ImGui::Text("Previous Regions Z-Up: %d", _previousRegionsZUp.size());
            ImGui::Text("Latest Regions Z-Up: %d", _latestRegionsZUp.size());
            ImGui::Text("Matches Found: %d", _matches.size());
            ImGui::Text("Total Map Regions: %d", _mapRegions.GetRegions().size());
            ImGui::NewLine();

            ImGui::Text("Last Translation: %.3lf, %.3lf, %.3lf", eulerAnglesToReference.x(), eulerAnglesToReference.y(), eulerAnglesToReference.z());
            ImGui::Text("Last Rotation Euler Angles: %.3lf, %.3lf, %.3lf", translationToReference.x(), translationToReference.y(), translationToReference.z());
            ImGui::Text("Landmark Planes: %lu", _network->GetSLAMPlanes().GetPlanes().size());
            ImGui::NewLine();

            ImGui::Text("Next File: %s", (_directory + fileNames[_frameIndex]).c_str());
            ImGui::NewLine();

            ImGui::SliderFloat("Match IoU Threshold", &app.MATCH_IOU_THRESHOLD, 0.01f, 1.0f);
            ImGui::SliderFloat("Match Angular Threshold", &app.MATCH_ANGULAR_THRESHOLD, 0.01f, 1.0f);
            ImGui::SliderFloat("Match Distance Threshold", &app.MATCH_DIST_THRESHOLD, 0.01f, 8.0f);

            ImGui::Checkbox("Factor Graph Enabled", &app.FACTOR_GRAPH_ENABLED);

            if(ImGui::Button("Enable Matching"))
            {
               plotter2D = true;
            }
            if(plotter2D)
            {
               MatchPlanarRegionsToMap();
               _mesher->ClearLines();
               _mesher->GenerateLineMeshForRegions(_previousRegionsZUp, nullptr);
               _mesher->GenerateLineMeshForRegions(_latestRegionsZUp, nullptr);
               _mesher->GenerateMeshForMatches(_latestRegionsZUp, _previousRegionsZUp, _matches, nullptr);
            }

            if (ImGui::Button("Register Next Set"))
            {
               _regionCalculator->LoadRegions(_directory, fileNames, _frameIndex);

               Update(_regionCalculator->planarRegionList, _frameIndex + 1);

               TransformAndCopyRegions(_latestRegionsZUp, _mapRegions, _sensorToMapTransform);


               // _mesher->GenerateMeshForRegions(_latestRegionsZUp, nullptr);

               //               GeomTools::AppendMeasurementsToFile(_sensorToMapTransform.GetMatrix().cast<float>(), _matches, _directory + "matches.txt", _frameIndex, _frameIndex + 1);

//               _mesher->GenerateLineMeshForRegions(_previousRegionsZUp, nullptr, true);
//               _mesher->GenerateLineMeshForRegions(_latestRegionsZUp, nullptr);
//               _mesher->GenerateMeshForMatches(_latestRegionsZUp, _previousRegionsZUp, _matches, nullptr);

               _mesher->GenerateLineMeshForRegions(_mapRegions.GetRegions(), nullptr);

               _previousRegionsZUp = std::move(_latestRegionsZUp);


               //               _mesher->GeneratePoseMesh(_sensorToMapTransform.GetInverse().GetMatrix().cast<float>(), nullptr);
               _frameIndex++;
            }

            if (ImGui::Button("Plot"))
            {
               plotter2D = true;
            }

            if (plotter2D)
            {
               _previousRegionsZUp[regionSelected]->ComputeBoundaryVerticesPlanar();
               const std::vector<Eigen::Vector2f>& boundaryPoints2D = _previousRegionsZUp[regionSelected]->GetPlanarPatchCentroids();

               //               _latestRegionsZUp[regionSelected]->ProjectToPlane(_previousRegionsZUp[regionSelected]->GetPlane());
               _latestRegionsZUp[regionSelected]->ComputeBoundaryVerticesPlanar();
               const std::vector<Eigen::Vector2f>& boundaryPointsSecond2D = _latestRegionsZUp[regionSelected]->GetPlanarPatchCentroids();

               auto intersection = HullTools::CalculateIntersection(boundaryPoints2D, boundaryPointsSecond2D);

               float IoU = HullTools::ComputeBoundingBoxIoU({boundaryPoints2D}, {boundaryPointsSecond2D});

               ImGui::Text("IoU for Hulls: %.2lf", IoU);

               if (ImGuiTools::BeginPlotWindow("Plotter 2D"))
               {
                  ImGuiTools::ScatterPlotXY(boundaryPoints2D, "Region 1");
                  ImGuiTools::ScatterPlotXY(boundaryPointsSecond2D, "Region 2");
                  ImGuiTools::EndPlotWindow();
               }
            }

            ImGui::EndTabItem();
         }
         ImGui::EndTabBar();
      }
      ImGui::EndTabItem();
   }
}

void MapHandler::Update(std::vector<std::shared_ptr<PlanarRegion>>& regions, int index)
{
   latestRegions = regions;

   if (_previousRegionsZUp.size() == 0)
   {
      TransformAndCopyRegions(latestRegions, _previousRegionsZUp, _transformZUp);
   }

   TransformAndCopyRegions(latestRegions, _latestRegionsZUp, _transformZUp);

   MatchPlanarRegionsToMap();

   if (_matches.size() > 0)
   {
      RegisterRegionsPointToPlane(1);

      Eigen::Quaterniond quaternion = _sensorToMapTransform.GetQuaternion();
      Eigen::Vector3d position = _sensorToMapTransform.GetTranslation();
      MS_INFO("Sensor To Map Pose ({}): {} {} {} {} {} {} {}", index, position.x(), position.y(), position.z(), quaternion.x(), quaternion.y(), quaternion.z(),
              quaternion.w());

      std::cout << _sensorToMapTransform.GetMatrix() << std::endl;

      if (_app.FACTOR_GRAPH_ENABLED)
      {
         _network->PublishPoseStamped(_sensorPoseRelative, index);
         _network->PublishPlanes(_latestRegionsZUp, index);
      }
      TransformAndCopyRegions(_latestRegionsZUp, _regionsInMapFrame, _sensorToMapTransform);
   }

   int currentPoseId = 1;
}

void MapHandler::MatchPlanarRegionsToMap()
{
   _matches.clear();
   for (int i = 0; i < _previousRegionsZUp.size(); i++)
   {
      if (_previousRegionsZUp[i]->GetNumOfBoundaryVertices() > 8)
      {
         for (int j = 0; j < _latestRegionsZUp.size(); j++)
         {
            if (_latestRegionsZUp[j]->GetNumOfBoundaryVertices() > 8)
            {
               Eigen::Vector3f prevNormal = _previousRegionsZUp[i]->GetNormal();
               Eigen::Vector3f curNormal = _latestRegionsZUp[j]->GetNormal();
               float angularDiff = fabs(prevNormal.dot(curNormal));

               Eigen::Vector3f prevCenter = _previousRegionsZUp[i]->GetCenter();
               Eigen::Vector3f curCenter = _latestRegionsZUp[j]->GetCenter();
               float dist = (curCenter - prevCenter).dot(prevNormal);
               //         float dist = fabs((prevCenter - curCenter).dot(curNormal)) + fabs((curCenter - prevCenter).dot(prevNormal));

               int countDiff = abs(_previousRegionsZUp[i]->GetNumOfBoundaryVertices() - _latestRegionsZUp[j]->GetNumOfBoundaryVertices());
               int maxCount = std::max(_previousRegionsZUp[i]->GetNumOfBoundaryVertices(), _latestRegionsZUp[j]->GetNumOfBoundaryVertices());


               std::shared_ptr<PlanarRegion> projectedRegion = std::make_shared<PlanarRegion>(_latestRegionsZUp[j]->getId());
               _latestRegionsZUp[j]->TransformAndFill(projectedRegion, {});
               projectedRegion->ProjectToPlane(_previousRegionsZUp[i]->GetPlane());

               projectedRegion->ComputeBoundaryVerticesPlanar();
               _previousRegionsZUp[i]->ComputeBoundaryVerticesPlanar();

               float IoU = HullTools::ComputeBoundingBoxIoU({_previousRegionsZUp[i]->GetPlanarPatchCentroids()},{projectedRegion->GetPlanarPatchCentroids()});


               if (dist < _app.MATCH_DIST_THRESHOLD && angularDiff > _app.MATCH_ANGULAR_THRESHOLD && IoU > _app.MATCH_IOU_THRESHOLD
                  //                     && ((float) countDiff / ((float) maxCount)) * 100.0f < _app.MATCH_PERCENT_VERTEX_THRESHOLD
                     )
               {
                  _matches.emplace_back(i, j);
                  if (_previousRegionsZUp[i]->getId() == -1 && _latestRegionsZUp[j]->getId() == -1)
                  {
                     _latestRegionsZUp[j]->setId(uniqueLandmarkCounter);
                     _previousRegionsZUp[j]->setId(uniqueLandmarkCounter);
                     uniqueLandmarkCounter++;
                  } else
                  {
                     _latestRegionsZUp[j]->setId(_previousRegionsZUp[i]->getId());
                  }
                  MS_INFO("Copying Region ID: {}", _previousRegionsZUp[i]->getId());
                  _previousRegionsZUp[i]->SetNumOfMeasurements(_previousRegionsZUp[i]->GetNumOfMeasurements() + 1);
                  _latestRegionsZUp[j]->SetNumOfMeasurements(_latestRegionsZUp[j]->GetNumOfMeasurements() + 1);
                  break;
               }
            }
         }
      }
   }
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
         Eigen::Vector3f correspondingMapCentroid = _previousRegionsZUp[_matches[m].first]->GetCenter();
         Eigen::Vector3f correspondingMapNormal = _previousRegionsZUp[_matches[m].first]->GetNormal();
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
   _sensorPoseRelative.SetAnglesAndTranslation(eulerAnglesToReference, translationToReference);
   _sensorToMapTransform.MultiplyRight(_sensorPoseRelative);
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
         Eigen::Vector3f correspondingMapCentroid = _previousRegionsZUp[_matches[m].first]->GetCenter();
         Eigen::Vector3f correspondingMapNormal = _previousRegionsZUp[_matches[m].first]->GetNormal();
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
   _sensorPoseRelative.SetAnglesAndTranslation(eulerAnglesToReference, translationToReference);
   _sensorToMapTransform.MultiplyRight(_sensorPoseRelative);
}

void MapHandler::InsertMapRegions(const std::vector<std::shared_ptr<PlanarRegion>>& regions)
{
   for (auto region: regions)
   {
      _mapRegions.InsertRegion(std::move(region), region->getId());
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

void MapHandler::UpdateMapLandmarks(const PlaneSet3D& planeSet)
{
   for (auto plane: planeSet.GetPlanes())
   {
      MS_INFO("Exists -> ID: {} Plane: {}", plane.first, plane.second.GetString());
      if (_mapRegions.Exists(plane.first))
      {
         _mapRegions.GetRegions()[plane.first]->ProjectToPlane(plane.second.GetParams().cast<float>());
      }
   }
}

void MapHandler::setDirectory(const std::string& directory)
{
   this->_directory = directory;
}

void MapHandler::TransformAndCopyRegions(const std::vector<std::shared_ptr<PlanarRegion>>& srcRegions, std::vector<std::shared_ptr<PlanarRegion>>& dstRegions,
                                         const RigidBodyTransform& transform)
{
   dstRegions.clear();
   for (int i = 0; i < srcRegions.size(); i++)
   {
      std::shared_ptr<PlanarRegion> planarRegion = std::make_shared<PlanarRegion>(srcRegions[i]->getId());
      srcRegions[i]->TransformAndFill(planarRegion, transform);
      dstRegions.emplace_back(std::move(planarRegion));
   }
}

void MapHandler::TransformAndCopyRegions(const std::vector<std::shared_ptr<PlanarRegion>>& srcRegions, PlanarRegionSet& dstRegionSet,
                                         const RigidBodyTransform& transform)
{
   dstRegionSet.GetRegions().clear();
   for (int i = 0; i < srcRegions.size(); i++)
   {
      std::shared_ptr<PlanarRegion> planarRegion = std::make_shared<PlanarRegion>(srcRegions[i]->getId());
      srcRegions[i]->TransformAndFill(planarRegion, transform);
      dstRegionSet.InsertRegion(std::move(planarRegion), planarRegion->getId());
   }
}




