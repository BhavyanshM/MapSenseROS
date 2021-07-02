//
// Created by quantum on 2/16/21.
//

#ifndef APPUTILS_H
#define APPUTILS_H

#include "ApplicationState.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include <opencv2/imgproc.hpp>

#include "GeomTools.h"
#include "MapsenseHeaders.h"
#include "PlanarRegion.h"

using namespace std;
using namespace cv;

typedef Eigen::Matrix<bool, Dynamic, Dynamic> BoolDynamicMatrix;

class AppUtils
{
   public:

      vector<Mat> images;
      Mat displayOutput;
      Mat debugOutput;
      uint16_t rows, cols;

      void display(uint16_t delay);

      void displayPointSet2D(vector<Vector2f> points, Vector2f offset, int scale);

      void setDisplayResolution(uint16_t rows, uint16_t cols);

      void canvasToMat(BoolDynamicMatrix canvas, Vector2i windowPos, uint8_t windowSize);

      void displayCanvasWithWindow(BoolDynamicMatrix canvas, Vector2i windowPos, uint8_t windowSize);

      static void
      capture_data(String projectPath, String filename, Mat depth, Mat color, Mat filteredDepth, Mat debug, ApplicationState appState, vector<shared_ptr<PlanarRegion>> regions);

      static void getFileNames(string dirName, vector<string>& files, bool printList = false);

      void appendToDebugOutput(Mat disp);

      void displayDebugOutput(ApplicationState appState);

      static void checkMemoryLimits();

      void clearDebug();

      static void DisplayImage(Mat disp, const ApplicationState& app);

};

#endif //APPUTILS_H
