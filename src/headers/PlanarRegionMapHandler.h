#ifndef PLANARREGIONMAPHANDLER_H
#define PLANARREGIONMAPHANDLER_H

#include "MapsenseHeaders.h"

#include <PlanarRegion.h>
#include "GeomTools.h"
#include "FactorGraphHandler.h"

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

      FactorGraphHandler* fgSLAM;
      vector<string> files;

      vector<shared_ptr<PlanarRegion>> regions, latestRegions, measuredRegions, mapRegions, regionsInMapFrame;

      vector<pair<int, int>> matches;
      vector<RigidBodyTransform> poses, atlasPoses;

      string directory;
      Vector3d translationToReference, eulerAnglesToReference;

      RigidBodyTransform _sensorToMapTransform;
      RigidBodyTransform _sensorPoseRelative;
      RigidBodyTransform _atlasSensorPose;
      RigidBodyTransform _atlasPreviousSensorPose;

      PlanarRegionMapHandler();

      void setDirectory(const string& directory);

      void matchPlanarRegionsToMap();

      void getFileNames(string dirName);

      void registerRegionsPointToPlane(uint8_t iterations);

      void registerRegionsPointToPoint();

      void mergeLatestRegions();

      void insertOrientedPlaneFactors(int currentPoseId);

      int insertOdometryFactor(RigidBodyTransform odometry);

      int insertPosePriorFactor(RigidBodyTransform pose);

      void setOrientedPlaneInitialValues();

      void extractFactorGraphLandmarks();

      void optimize();

      void printRefCounts();

      void transformAndCopyLatestRegions(vector<shared_ptr<PlanarRegion>>& transformedRegions, const RigidBodyTransform& transform);
};

#endif //PLANARREGIONMAPHANDLER_H
