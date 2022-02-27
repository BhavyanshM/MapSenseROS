#ifndef PLANARREGIONMAPHANDLER_H
#define PLANARREGIONMAPHANDLER_H

#include "PlanarRegionCalculator.h"
#include <PlanarRegion.h>
#include "GeomTools.h"
#include "FactorGraphHandler.h"

using namespace std;

class PlanarRegionMapHandler
{
   public:
      PlanarRegionMapHandler();

      void SetRegionCalculator(PlanarRegionCalculator* regionCalculator){ _regionCalculator = regionCalculator;}

      void setDirectory(const string& directory);

      void InsertMapRegions(const std::vector<shared_ptr<PlanarRegion>>& regions);

      void MatchPlanarRegionsToMap();

      void getFileNames(string dirName);

      void RegisterRegionsPointToPlane(uint8_t iterations);

      void RegisterRegionsPointToPoint();

      void MergeLatestRegions();

      void InsertOrientedPlaneFactors(int currentPoseId);

      int insertOdometryFactor(RigidBodyTransform odometry);

      int insertPosePriorFactor(RigidBodyTransform pose);

      void SetOrientedPlaneInitialValues();

      void ExtractFactorGraphLandmarks();

      void Optimize();

      void PrintRefCounts();

      void TransformAndCopyRegions(const vector<shared_ptr<PlanarRegion>>& srcRegions, vector<shared_ptr<PlanarRegion>>& dstRegions, const RigidBodyTransform& transform);

      void ImGuiUpdate();

   public:

      bool SLAM_ENABLED = false;

      bool plotter2D = false;
      float COMPRESS_DIST_THRESHOLD = 0.05f;
      float SEGMENT_DIST_THRESHOLD = 0.2f;
      float MATCH_DIST_THRESHOLD = 0.1f;
      float MATCH_ANGULAR_THRESHOLD = 0.9f;
      int MATCH_PERCENT_VERTEX_THRESHOLD = 20;
      bool FACTOR_GRAPH = true;
      bool ISAM2 = true;
      uint8_t ISAM2_NUM_STEPS = 4;

      FactorGraphHandler* fgSLAM;
      vector<string> files;

      vector<shared_ptr<PlanarRegion>> regions, latestRegions, measuredRegions, mapRegions, regionsInMapFrame, _latestRegionsZUp, _testLatestRegions;

      vector<pair<int, int>> matches;
      vector<RigidBodyTransform> poses, atlasPoses;

      string directory;
      Eigen::Vector3d translationToReference, eulerAnglesToReference;

      RigidBodyTransform _sensorToMapTransform;
      RigidBodyTransform _sensorPoseRelative;
      RigidBodyTransform _atlasSensorPose;
      RigidBodyTransform _atlasPreviousSensorPose;

      PlanarRegionCalculator* _regionCalculator;


};

#endif //PLANARREGIONMAPHANDLER_H
