#ifndef PLANARREGIONMAPHANDLER_H
#define PLANARREGIONMAPHANDLER_H

#include <PlanarRegion.h>
#include <GeomTools.h>
#include <dirent.h>

using namespace std;

class PlanarRegionMapHandler
{
   public:
      vector<string> files;
      vector<shared_ptr<PlanarRegion>> regions, latestRegions;
      vector<pair<int, int>> matches;
      string directory;
      Vector3d translationToReference, eulerAnglesToReference;

      void matchPlanarRegionstoMap(vector<shared_ptr<PlanarRegion>> latestRegions);

      void loadRegions(int frameId, vector<shared_ptr<PlanarRegion>>& regions);

      void getFileNames(string dirName);

      void registerRegions();

      void transformLatestRegions(Vector3d translation, Vector3d eulerAngles);

      void transformLatestRegions(Vector3d translation, Matrix3d rotation);

      void transformAndCopyLatestRegions(Vector3d translation, Matrix3d rotation, vector<shared_ptr<PlanarRegion>>& transformedRegions);
};

#endif //PLANARREGIONMAPHANDLER_H
