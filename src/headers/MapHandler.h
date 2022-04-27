#ifndef PLANARREGIONMAPHANDLER_H
#define PLANARREGIONMAPHANDLER_H

#include "PlanarRegionCalculator.h"
#include "GeomTools.h"
#include "HullTools.h"
#include "SLAMModule.h"
#include "MeshGenerator.h"
#include "NetworkManager.h"
#include "filesystem"
#include "ImGui/FileBrowserUI.h"

class PlanarRegion;

class MapHandler
{
   public:
      MapHandler(NetworkManager* network, ApplicationState& app);

      void Update(std::vector <std::shared_ptr<PlanarRegion>>& regions, int index);

      void SetRegionCalculator(PlanarRegionCalculator* regionCalculator){ _regionCalculator = regionCalculator;}

      void SetSLAMModule(SLAMModule* slamModule){ _slam = slamModule;}

      void setDirectory(const std::string& directory);

      void InsertMapRegions(const std::vector<std::shared_ptr<PlanarRegion>>& regions);

      void MatchPlanarRegionsToMap();

      void getFileNames(std::string dirName);

      void RegisterRegionsPointToPlane(uint8_t iterations);

      void RegisterRegionsPointToPoint();

      void MergeLatestRegions();

      void TransformAndCopyRegions(const std::vector<std::shared_ptr<PlanarRegion>>& srcRegions, PlanarRegionSet& dstRegionSet, const RigidBodyTransform& transform);

      void TransformAndCopyRegions(const std::vector<std::shared_ptr<PlanarRegion>>& srcRegions, std::vector<std::shared_ptr<PlanarRegion>>& dstRegions, const RigidBodyTransform& transform);

      void ImGuiUpdate(ApplicationState& appState);

      void SetMeshGenerator(MeshGenerator* mesher) {_mesher = mesher;}

      void UpdateMapLandmarks(const PlaneSet3D& planeSet);

      MeshGenerator* GetMesher() const { return _mesher; }

      void DebugPrint()
      {
         for(auto region : _latestRegionsZUp)
         {
            Eigen::Vector3f center = region->GetCenter();
            Eigen::Vector3f normal = region->GetNormal().normalized();
            MS_INFO("Region Params: {} {} {} {}", normal.x(), normal.y(), normal.z(), -center.dot(normal));
            Plane3D plane(region->GetCenter().x(), region->GetCenter().y(), region->GetCenter().z(),
                          region->GetNormal().x(), region->GetNormal().y(), region->GetNormal().z(), region->getId());
            MS_INFO("Plane Original: {}", plane.GetString());
            Plane3D planeInMapFrame = plane.GetTransformed(_sensorToMapTransform);
            MS_INFO("Original Plane Transformed: {}", planeInMapFrame.GetString());
         }

         for(auto region : _regionsInMapFrame)
         {
            Eigen::Vector3f center = region->GetCenter();
            Eigen::Vector3f normal = region->GetNormal().normalized();
            MS_INFO("Transformed Region Params: {} {} {} {}", normal.x(), normal.y(), normal.z(), -center.dot(normal));
            Plane3D plane(region->GetCenter().x(), region->GetCenter().y(), region->GetCenter().z(),
                          region->GetNormal().x(), region->GetNormal().y(), region->GetNormal().z(), region->getId());
            MS_INFO("Transformed Plane Original: {}", plane.GetString());
         }
      }

   public:

      bool SLAM_ENABLED = false;
      bool showFileBrowser = false;

      bool plotter2D = false;

      bool FACTOR_GRAPH = true;
      bool ISAM2 = true;
      uint8_t ISAM2_NUM_STEPS = 4;

      int _frameIndex = 0;


//      FactorGraphHandler* fgSLAM;
      std::vector<std::string> files;

      std::vector<std::shared_ptr<PlanarRegion>> _previousRegionsZUp, latestRegions, _latestRegionsZUp, _regionsInMapFrame;

      PlanarRegionSet _mapRegions;

      std::vector<std::pair<int, int>> _matches;
      std::vector<RigidBodyTransform> poses, atlasPoses;
      RigidBodyTransform _transformZUp;

      std::string _directory;
      Eigen::Vector3d translationToReference, eulerAnglesToReference;

      RigidBodyTransform _sensorToMapTransform;
      RigidBodyTransform _sensorPoseRelative;
      RigidBodyTransform _atlasSensorPose;
      RigidBodyTransform _atlasPreviousSensorPose;

      PlanarRegionCalculator* _regionCalculator;
      SLAMModule* _slam;
      MeshGenerator* _mesher;
      NetworkManager* _network;

      int regionSelected = 0;
      int fileSelected = 0;
      int segmentSelected = 0;
      int uniqueLandmarkCounter = 1;

      std::vector<std::string> fileNames;

   private:
      ApplicationState& _app;

      float endX = 0;
      float endY = 1;
      float endZ = 0;
      float endYaw = 0;
      float endPitch = 1;
      float endRoll = -1;

      Clay::FileBrowserUI _fileBrowserUI;

};

#endif //PLANARREGIONMAPHANDLER_H
