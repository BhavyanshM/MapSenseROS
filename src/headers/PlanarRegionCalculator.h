#ifndef SRC_PLANARREGIONCALCULATOR_H
#define SRC_PLANARREGIONCALCULATOR_H

#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "cv_bridge/cv_bridge.h"
#include <iostream>
#include <CL/cl.hpp>
#include <cmath>
#include <sstream>
#include "tbb/tbb.h"
#include <random>

#include "NetworkManager.h"
#include "MapFrame.h"
#include "MapFrameProcessor.h"
#include "PlanarRegion.h"

using namespace ros;
using namespace std;
using namespace chrono;
using namespace cv;

class PlanarRegionCalculator
{
   public:
      cl::Kernel filterKernel, packKernel, mergeKernel;
      cl::Context context;
      cl::CommandQueue commandQueue;
      cl::Event event;
      cl::size_t<3> origin;

      ApplicationState app;
      NetworkManager *_dataReceiver;

      Mat inputDepth;
      Mat inputColor;
      double inputTimestamp;
      Mat filteredDepth;
      Mat inputStereoLeft, inputStereoRight;

      MapFrame output;
      MapFrameProcessor mapFrameProcessor;
      vector<shared_ptr<PlanarRegion>> planarRegionList;

      explicit PlanarRegionCalculator(ApplicationState& app);

      void generatePatchGraph(ApplicationState appState);

      void initOpenCL(ApplicationState app);

      void launch_tester(ApplicationState appState);

      void generateRegions(NetworkManager *receiver, ApplicationState appState);

      void publishRegions(vector<shared_ptr<PlanarRegion>> regionList);

      void getFilteredDepth(Mat& dispDepth, ApplicationState appState);

      void getInputDepth(Mat& dispDepth, ApplicationState appState);

      static void onMouse(int event, int x, int y, int flags, void *userdata);
};

#endif //SRC_PLANARREGIONCALCULATOR_H
