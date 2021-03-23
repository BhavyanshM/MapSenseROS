//
// Created by quantum on 3/3/21.
//

#include "PlanarRegionMapTester.h"

bool PlanarRegionMapTester::runTests()
{
   bool success = true;

   if (!((GeomTools::getProjectedPoint(Vector4f(1, 0, 0, -1), Vector3f(0, 1, 1)) - Vector3f(1, 1, 1)).norm() < 1e-4f))
   {
      printf("Failed Test: 1\n");
      cout << GeomTools::getProjectedPoint(Vector4f(1, 0, 0, -1), Vector3f(0, 1, 1)) << endl;
      success = true;
   }
   if (!((GeomTools::getProjectedPoint(Vector4f(1, 1, 0, -1), Vector3f(-6, -6, 1)) - Vector3f(0.5f, 0.5f, 1)).norm() < 1e-2f))
   {
      printf("Failed Test: 2\n");
      cout << GeomTools::getProjectedPoint(Vector4f(1, 1, 0, -1), Vector3f(-6, -6, 1)) << endl;
      success = false;
   }

   return success;
}
