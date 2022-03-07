#ifndef PLANARREGIONCALCULATOR_H
#define PLANARREGIONCALCULATOR_H


#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "cv_bridge/cv_bridge.h"
#include <CL/cl.hpp>
#include <map_sense/RawGPUPlanarRegionList.h>

#include "MapFrame.h"
#include "MapFrameProcessor.h"
#include "PlanarRegion.h"
#include "OpenCLManager.h"
#include "AppUtils.h"

using namespace ros;
using namespace std;
using namespace chrono;

class PlanarRegionCalculator
{
   public:
      PlanarRegionCalculator(int argc, char **argv, ApplicationState& app);

      void Render();

      void LoadRegions(std::string path, std::vector<std::string>& fileNames, int index);

      void ImGuiUpdate(ApplicationState& appState);

      bool GeneratePatchGraphFromDepth(ApplicationState& appState);

      void generateRegionsFromDepth(ApplicationState& appState, cv::Mat& depth, double inputTimestamp);

      map_sense::RawGPUPlanarRegionList publishRegions();

      void getFilteredDepth(cv::Mat& dispDepth, ApplicationState& appState);

      static void onMouse(int event, int x, int y, int flags, void *userdata);

      void setOpenCLManager(OpenCLManager* ocl) {_openCL = ocl;}

      uint8_t CreateParameterBuffer(const ApplicationState& app);

      void GeneratePatchGraphFromPointCloud(ApplicationState& appState, std::vector<float>& points, double inputTimestamp);

      void GenerateRegionFromPointcloudOnCPU();


   public:
      //      cl::Kernel filterKernel, packKernel, mergeKernel;
      //      cl::Context context;
      //      cl::CommandQueue commandQueue;
      //      cl::Event event;

      cl::size_t<3> origin;

      ApplicationState& app;
      AppUtils appUtils;

      cv::Mat inputDepth;
      cv::Mat inputColor;
      double inputTimestamp;
      cv::Mat filteredDepth;
      cv::Mat inputStereoLeft, inputStereoRight;

      MapFrame output;
      MapFrameProcessor* _mapFrameProcessor;
      OpenCLManager* _openCL;
      vector<shared_ptr<PlanarRegion>> planarRegionList;
      map_sense::RawGPUPlanarRegionList _planarRegionsToPublish;

      std::vector<std::string> depthFiles, cloudFiles;
      int depthFileSelected = 0;
      int cloudFileSelected = 0;

      int frameId = 0;
      int depthReceiverId = -1;
      bool _render = true;

      bool RenderEnabled();
};

#endif //PLANARREGIONCALCULATOR_H
