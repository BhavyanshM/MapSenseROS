#ifndef PLANARREGIONMAPHANDLER_H
#define PLANARREGIONMAPHANDLER_H

#include "MapsenseHeaders.h"
#include <PlanarRegion.h>
#include "../Geometry/include/GeomTools.h"
#include "../SLAM/FactorGraphSLAM.h"

using namespace std;

class PlanarRegionMapHandler
{
   public:

      float MATCH_DIST_THRESHOLD = 0.1f;
      float MATCH_ANGULAR_THRESHOLD = 0.9f;
      int MATCH_PERCENT_VERTEX_THRESHOLD = 20;
      bool FACTOR_GRAPH = false;
      bool ISAM2 = false;
      uint8_t ISAM2_NUM_STEPS = 4;

      FactorGraphSLAM fgSLAM;
      vector<string> files;
      vector<shared_ptr<PlanarRegion>> regions, latestRegions, measuredRegions, mapRegions;
      vector<pair<int, int>> matches;
      vector<RigidBodyTransform> poses;

      string directory;
      Vector3d translationToReference, eulerAnglesToReference;

      RigidBodyTransform _sensorToMapTransform;
      RigidBodyTransform _sensorPoseRelative;

      void setDirectory(const string& directory);

      void matchPlanarRegionsToMap(vector<shared_ptr<PlanarRegion>> latestRegions);

      void loadRegions(int frameId, vector<shared_ptr<PlanarRegion>>& regions);

      void getFileNames(string dirName);

      void registerRegionsPointToPlane();

      void registerRegionsPointToPoint();

      void transformLatestRegions(RigidBodyTransform transform);

      void transformLatestRegions(Vector3d translation, Matrix3d rotation);

      void transformAndCopyLatestRegions(RigidBodyTransform transform, vector<shared_ptr<PlanarRegion>>& transformedRegions);

      void mergeLatestRegions();

      void updateFactorGraphLandmarks(vector<shared_ptr<PlanarRegion>>& regionsToInsert, int currentPoseId);

      int updateFactorGraphPoses(RigidBodyTransform odometry);

      void initFactorGraphState(RigidBodyTransform sensorPose, vector<shared_ptr<PlanarRegion>> regionInMapFrame);

      void updateMapRegionsWithSLAM();
};

#endif //PLANARREGIONMAPHANDLER_H
