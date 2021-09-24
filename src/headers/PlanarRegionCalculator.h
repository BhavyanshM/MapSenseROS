#ifndef PLANARREGIONCALCULATOR_H
#define PLANARREGIONCALCULATOR_H


#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "cv_bridge/cv_bridge.h"
#include <CL/cl.hpp>

#include "MapsenseHeaders.h"
#include "NetworkManager.h"
#include "MapFrame.h"
#include "MapFrameProcessor.h"
#include "PlanarRegion.h"
#include "OpenCLManager.h"

using namespace ros;
using namespace std;
using namespace chrono;

class PlanarRegionCalculator
{
   public:
//      cl::Kernel filterKernel, packKernel, mergeKernel;
//      cl::Context context;
//      cl::CommandQueue commandQueue;
//      cl::Event event;

      cl::size_t<3> origin;

      ApplicationState app;
      AppUtils appUtils;
      NetworkManager *_dataReceiver;

      cv::Mat inputDepth;
      cv::Mat inputColor;
      double inputTimestamp;
      cv::Mat filteredDepth;
      cv::Mat inputStereoLeft, inputStereoRight;

      MapFrame output;
      MapFrameProcessor _mapFrameProcessor;
      OpenCLManager* _openCL;
      vector<shared_ptr<PlanarRegion>> planarRegionList;

      int frameId = 0;
      int depthReceiverId = -1;

      explicit PlanarRegionCalculator(int argc, char** argv, NetworkManager* network, ApplicationState& app);

      void render();

      bool generatePatchGraph(ApplicationState appState);

      bool generatePatchGraphFromStereo(ApplicationState& appState);

      void ImGuiUpdate(ApplicationState& appState);

//      void initOpenCL();

      void generateRegionsFromDepth(ApplicationState appState);

      void generateRegionsFromStereo(ApplicationState appState);

      void publishRegions(vector<shared_ptr<PlanarRegion>> regionList);

      void getFilteredDepth(cv::Mat& dispDepth, ApplicationState appState);

      static void onMouse(int event, int x, int y, int flags, void *userdata);

      void setOpenCLManager(OpenCLManager* ocl) {_openCL = ocl;}

      void publish() {publishRegions(planarRegionList);}

      uint8_t loadParameters(const ApplicationState& app);

};

#endif //PLANARREGIONCALCULATOR_H
