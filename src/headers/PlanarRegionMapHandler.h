//
// Created by quantum on 2/8/21.
//

#ifndef PLOTTER3D_PY_PLANARREGIONMAPHANDLER_H
#define PLOTTER3D_PY_PLANARREGIONMAPHANDLER_H


#include <PlanarRegion.h>

class PlanarRegionMapHandler {
public:
    vector<shared_ptr<PlanarRegion>> regions, latestRegions;

    void registerRegions(vector<shared_ptr<PlanarRegion>> latestRegions);
    void tester();
    void loadRegions();
};


#endif //PLOTTER3D_PY_PLANARREGIONMAPHANDLER_H
