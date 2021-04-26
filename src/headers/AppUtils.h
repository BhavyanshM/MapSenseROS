//
// Created by quantum on 2/16/21.
//

#ifndef APPUTILS_H
#define APPUTILS_H

#include <iostream>
#include "ApplicationState.h"
#include "sys/resource.h"
#include <dirent.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "PlanarRegion.h"

using namespace std;
using namespace cv;

class AppUtils
{
   public:

      vector<Mat> images;
      Mat debugOutput;

      static void
      capture_data(String projectPath, String filename, Mat depth, Mat color, Mat filteredDepth, Mat debug, ApplicationState appState, vector<shared_ptr<PlanarRegion>> regions);

      static void getFileNames(string dirName, vector<string>& files);

      void appendToDebugOutput(Mat disp);

      void displayDebugOutput(ApplicationState appState);

      static void checkMemoryLimits();

      static void write_regions(vector<shared_ptr<PlanarRegion>> regions, string fileName);
};

#endif //APPUTILS_H
