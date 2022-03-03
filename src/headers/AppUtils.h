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

typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> BoolDynamicMatrix;

class AppUtils
{
   public:

      vector<cv::Mat> images;
      cv::Mat displayOutput;
      cv::Mat debugOutput;
      uint16_t rows, cols;

      void display(uint16_t delay);

      void displayPointSet2D(vector<Eigen::Vector2f> points, Eigen::Vector2f offset, int scale);

      void setDisplayResolution(uint16_t rows, uint16_t cols);

      void canvasToMat(BoolDynamicMatrix canvas, Eigen::Vector2i windowPos, uint8_t windowSize);

      void displayCanvasWithWindow(BoolDynamicMatrix canvas, Eigen::Vector2i windowPos, uint8_t windowSize);

      static void
      capture_data(std::string projectPath, std::string filename, cv::Mat depth, cv::Mat color, cv::Mat filteredDepth, cv::Mat debug, ApplicationState appState, vector<shared_ptr<PlanarRegion>> regions);

      static void getFileNames(string dirName, vector<string>& files, bool printList = false);

      void appendToDebugOutput(cv::Mat disp);

      void displayDebugOutput(ApplicationState appState);

      static void checkMemoryLimits();

      void clearDebug();

      static void DisplayImage(cv::Mat disp, const ApplicationState& app);

      static void PrintMat(cv::Mat& mat, int value = -1, bool invert = false, int rowLimit = 0, int colLimit = 0);

      static void CalculateAndPrintStatsMat(cv::Mat& mat);

};

#endif //APPUTILS_H
