//
// Created by quantum on 2/16/21.
//

#ifndef PLOTTER3D_PY_APPUTILS_H
#define PLOTTER3D_PY_APPUTILS_H

#include <iostream>
#include "PlanarRegionCalculator.h"
#include "ApplicationState.h"
#include "sys/resource.h"

using namespace std;
using namespace cv;

class AppUtils
{
   public:
      static void
      capture_data(String filename, Mat depth, Mat color, Mat filteredDepth, Mat debug, ApplicationState appState, vector<shared_ptr<PlanarRegion>> regions);

      static void displayDebugOutput(Mat disp);

      static void checkMemoryLimits();

      static void write_regions(vector<shared_ptr<PlanarRegion>> regions, int frameId);
};

#endif //PLOTTER3D_PY_APPUTILS_H
