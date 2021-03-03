#ifndef PLANARREGIONMAPHANDLER_H
#define PLANARREGIONMAPHANDLER_H

#include <PlanarRegion.h>
#include <dirent.h>

using namespace std;

class PlanarRegionMapHandler
{
   public:
      vector<string> files;
      vector<shared_ptr<PlanarRegion>> regions, latestRegions;
      vector<pair<int, int>> matches;
      string directory;

      void matchPlanarRegionstoMap(vector<shared_ptr<PlanarRegion>> latestRegions);

      void loadRegions(int frameId, vector<shared_ptr<PlanarRegion>>& regions);

      void getFileNames(string dirName);

      Matrix4f registerRegions();
};

#endif //PLANARREGIONMAPHANDLER_H
