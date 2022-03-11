#ifndef PLANARREGIONMAPTESTER_H
#define PLANARREGIONMAPTESTER_H

#include <MapHandler.h>
#include "../Geometry/include/KDTree.h"
#include "gtest/gtest.h"

class PlanarRegionMapTester
{
   public:
      PlanarRegionMapHandler mapper;
      static bool runTests(int argc, char** argv);
      void testKDTree();
};

#endif //PLANARREGIONMAPTESTER_H
