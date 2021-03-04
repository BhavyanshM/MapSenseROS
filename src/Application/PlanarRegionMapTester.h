#ifndef PLANARREGIONMAPTESTER_H
#define PLANARREGIONMAPTESTER_H

#include <PlanarRegionMapHandler.h>

class PlanarRegionMapTester
{
   public:
      PlanarRegionMapHandler mapper;
      static bool testICPTransformedPair();
};

#endif //PLANARREGIONMAPTESTER_H
