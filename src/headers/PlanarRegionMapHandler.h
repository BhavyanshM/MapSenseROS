#ifndef PLANARREGIONMAPHANDLER_H
#define PLANARREGIONMAPHANDLER_H

#include <PlanarRegion.h>
#include <GeomTools.h>
#include <dirent.h>
#include <algorithm>
#include "../SLAM/FactorGraphSLAM.h"

using namespace std;

class PlanarRegionMapHandler
{
   public:
      FactorGraphSLAM fgSLAM;
      vector<string> files;
      vector<shared_ptr<PlanarRegion>> regions, latestRegions, measuredRegions, mapRegions;
      vector<pair<int, int>> matches;
      vector<MatrixXd> poses;

      string directory;
      Vector3d translationToReference, eulerAnglesToReference;
      MatrixXd mapToSensorTransform = Eigen::MatrixXd::Identity(4, 4);
      MatrixXd sensorPoseRelative = Eigen::MatrixXd::Identity(4, 4);

      void matchPlanarRegionsToMap(vector<shared_ptr<PlanarRegion>> latestRegions);

      void loadRegions(int frameId, vector<shared_ptr<PlanarRegion>>& regions);

      void getFileNames(string dirName);

      void registerRegions();

      void transformLatestRegions(Vector3d translation, Vector3d eulerAngles);

      void transformLatestRegions(Vector3d translation, Matrix3d rotation);

      void transformAndCopyLatestRegions(Vector3d translation, Matrix3d rotation, vector<shared_ptr<PlanarRegion>>& transformedRegions);

      void mergeLatestRegions();

      void updateFactorGraphLandmarks(vector<shared_ptr<PlanarRegion>>& regionsToInsert, int currentPoseId);

      int updateFactorGraphPoses();

      void initFactorGraphState();

      void updateMapRegions();
};

#endif //PLANARREGIONMAPHANDLER_H
