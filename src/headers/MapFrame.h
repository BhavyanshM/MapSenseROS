#ifndef SRC_MAPFRAME_H
#define SRC_MAPFRAME_H

#include "opencv2/core/core.hpp"
#include <opencv2/imgproc.hpp>
#include "ApplicationState.h"
#include "MapsenseHeaders.h"

using namespace std;

class MapFrame
{
   public:

      cv::Mat regionOutput;
      cv::Mat patchData;

      void setRegionOutput(cv::Mat& regionOutput);

      void setPatchData(cv::Mat& patchData);

      cv::Mat& getRegionOutput();

      cv::Mat& getPatchData();

      void drawGraph(cv::Mat& img, ApplicationState app);

      cv::Vec6f getPatch(int x, int y);
};

#endif //SRC_MAPFRAME_H
