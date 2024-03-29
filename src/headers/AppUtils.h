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
#include "PlanarRegion.h"

typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> BoolDynamicMatrix;

class AppUtils
{
   public:

      std::vector<cv::Mat> images;
      cv::Mat displayOutput;
      cv::Mat debugOutput;
      uint16_t rows, cols;

      void display(uint16_t delay);

      void displayPointSet2D(std::vector<Eigen::Vector2f> points, Eigen::Vector2f offset, int scale);

      void setDisplayResolution(uint16_t rows, uint16_t cols);

      void canvasToMat(BoolDynamicMatrix canvas, Eigen::Vector2i windowPos, uint8_t windowSize);

      void displayCanvasWithWindow(BoolDynamicMatrix canvas, Eigen::Vector2i windowPos, uint8_t windowSize);

      static void
      capture_data(std::string projectPath, std::string filename, cv::Mat depth, cv::Mat color, cv::Mat filteredDepth, cv::Mat debug, ApplicationState appState, std::vector<std::shared_ptr<PlanarRegion>> regions);

      static void getFileNames(std::string dirName, std::vector<std::string>& files, bool printList = false);

      void appendToDebugOutput(cv::Mat disp);

      void displayDebugOutput(ApplicationState appState);

      static void checkMemoryLimits();

      void clearDebug();

      static void DisplayImage(cv::Mat disp, const ApplicationState& app, const std::string& title = "DisplayWindow");

      static void PrintMatR8(cv::Mat& mat, int value = -1, bool invert = false, bool constant = false, int rowLimit = 0, int colLimit = 0);

      static void PrintMatR16(cv::Mat& mat, int value = -1, bool invert = false, int rowLimit = 0, int colLimit = 0, bool linear = false);

      static void CalculateAndPrintStatsMat(cv::Mat& mat);

};

#endif //APPUTILS_H
