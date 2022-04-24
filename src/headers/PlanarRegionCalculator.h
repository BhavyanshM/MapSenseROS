#ifndef PLANARREGIONCALCULATOR_H
#define PLANARREGIONCALCULATOR_H


#include "geometry_msgs/PoseStamped.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "cv_bridge/cv_bridge.h"
#include <CL/cl.hpp>
#include <map_sense/RawGPUPlanarRegionList.h>

#include "MapFrameProcessor.h"
//#include "MapFrame.h"
//#include "PlanarRegion.h"
#include "OpenCLManager.h"
#include "AppUtils.h"

class MapFrame;
class PlanarRegion;

#include "Scene/Mesh/PointCloud.h"

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

      void GeneratePatchGraphFromPointCloud(ApplicationState& appState, const Clay::Ref<Clay::PointCloud>& cloud, double inputTimestamp);

      void GenerateRegionFromPointcloudOnCPU();

      bool RenderEnabled();


      std::vector<std::shared_ptr<PlanarRegion>> planarRegionList;
      std::vector<std::shared_ptr<PlanarRegion>> _hashRegionsZUp;
      std::vector<std::shared_ptr<PlanarRegion>> _depthRegionsZUp;
      map_sense::RawGPUPlanarRegionList _planarRegionsToPublish;

   private:
      //      cl::Kernel filterKernel, packKernel, mergeKernel;
      //      cl::Context context;
      //      cl::CommandQueue commandQueue;
      //      cl::Event event;

      std::unordered_map<std::string, cv::Mat> channelMap;

      cl::size_t<3> origin;
      ApplicationState& app;

      AppUtils appUtils;
      cv::Mat inputDepth;
      cv::Mat inputColor;
      double inputTimestamp;
      cv::Mat filteredDepth;

      cv::Mat inputStereoLeft, inputStereoRight;
      MapFrame output;
      MapFrameProcessor* _depthMapFrameProcessor;
      MapFrameProcessor* _hashMapFrameProcessor;

      OpenCLManager* _openCL;

      RigidBodyTransform _headToL515Transform;
      RigidBodyTransform _headToOusterTransform;
      RigidBodyTransform _transformZUp;

      std::vector<std::string> depthFiles, cloudFiles;
      int depthFileSelected = 0;
      int cloudFileSelected = 0;

      int frameId = 0;
      int depthReceiverId = -1;
      bool _render = true;

      float xAngle = 0;
      float yAngle = 0;
      float zAngle = 0;
      float xOffset = 0;
      float yOffset = 0;
      float zOffset = 0;


};

#endif //PLANARREGIONCALCULATOR_H
