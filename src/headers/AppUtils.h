//
// Created by quantum on 2/16/21.
//

#ifndef APPUTILS_H
#define APPUTILS_H

#include <iostream>
#include "PlanarRegionCalculator.h"
#include "ApplicationState.h"
#include "sys/resource.h"

using namespace std;
using namespace cv;

class AppUtils
{
   public:

      vector<Mat> images;
      Mat debugOutput;

      static void
      capture_data(String filename, Mat depth, Mat color, Mat filteredDepth, Mat debug, ApplicationState appState, vector<shared_ptr<PlanarRegion>> regions);

      void appendToDebugOutput(Mat disp);

      void displayDebugOutput(ApplicationState appState);

      static void checkMemoryLimits();

      static void write_regions(vector<shared_ptr<PlanarRegion>> regions, int frameId);
};

#endif //APPUTILS_H
