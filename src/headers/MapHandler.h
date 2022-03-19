#ifndef PLANARREGIONMAPHANDLER_H
#define PLANARREGIONMAPHANDLER_H

#include "PlanarRegionCalculator.h"
#include "PlanarRegion.h"
#include "GeomTools.h"
#include "SLAMModule.h"
#include "MeshGenerator.h"

class MapHandler
{
   public:
      MapHandler(ApplicationState& app);

      void Update(std::vector <std::shared_ptr<PlanarRegion>>& regions);

      void SetRegionCalculator(PlanarRegionCalculator* regionCalculator){ _regionCalculator = regionCalculator;}

      void SetSLAMModule(SLAMModule* slamModule){ _slam = slamModule;}

      void setDirectory(const std::string& directory);

      void InsertMapRegions(const std::vector<std::shared_ptr<PlanarRegion>>& regions);

      void MatchPlanarRegionsToMap();

      void getFileNames(std::string dirName);

      void RegisterRegionsPointToPlane(uint8_t iterations);

      void RegisterRegionsPointToPoint();

      void MergeLatestRegions();

//      void InsertOrientedPlaneFactors(int currentPoseId);
//
//      int insertOdometryFactor(RigidBodyTransform odometry);
//
//      int insertPosePriorFactor(RigidBodyTransform pose);
//
//      void SetOrientedPlaneInitialValues();
//
//      void ExtractFactorGraphLandmarks();
//
//      void Optimize();

      void PrintRefCounts();

      void TransformAndCopyRegions(const std::vector<std::shared_ptr<PlanarRegion>>& srcRegions, std::vector<std::shared_ptr<PlanarRegion>>& dstRegions, const RigidBodyTransform& transform);

      void ImGuiUpdate(ApplicationState& appState);

      void SetMeshGenerator(MeshGenerator* mesher) {_mesher = mesher;}

   public:

      bool SLAM_ENABLED = false;

      bool plotter2D = false;


      bool FACTOR_GRAPH = true;
      bool ISAM2 = true;
      uint8_t ISAM2_NUM_STEPS = 4;

      int _frameIndex = 0;

//      FactorGraphHandler* fgSLAM;
      std::vector<std::string> files;

      std::vector<std::shared_ptr<PlanarRegion>> _regions, latestRegions, measuredRegions, _mapRegions, regionsInMapFrame, _latestRegionsZUp, _testLatestRegions;

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

      int regionSelected = 0;
      int fileSelected = 0;
      int segmentSelected = 0;
      std::vector<std::string> fileNames;

   private:
      ApplicationState& _app;
};

#endif //PLANARREGIONMAPHANDLER_H
