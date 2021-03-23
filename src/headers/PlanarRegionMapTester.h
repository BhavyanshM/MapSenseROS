#ifndef PLANARREGIONMAPTESTER_H
#define PLANARREGIONMAPTESTER_H

#include <PlanarRegionMapHandler.h>
#include "gtest/gtest.h"

class PlanarRegionMapTester
{
   public:
      PlanarRegionMapHandler mapper;
      static bool runTests(int argc, char** argv);
};

#endif //PLANARREGIONMAPTESTER_H
