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
      string directory;

      void registerRegions(vector<shared_ptr<PlanarRegion>> latestRegions);

      void loadRegion(int frameId, shared_ptr<PlanarRegion> region);

      void getFileNames(string dirName);
};

#endif //PLANARREGIONMAPHANDLER_H
