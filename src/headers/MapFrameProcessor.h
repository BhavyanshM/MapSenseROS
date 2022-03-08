#ifndef SRC_MAPFRAMEPROCESSOR_H
#define SRC_MAPFRAMEPROCESSOR_H

#include "Core.h"
#include "MapFrame.h"
#include "PlanarRegion.h"
#include <opencv2/highgui.hpp>
#include "ApplicationState.h"
#include "MapsenseHeaders.h"

using namespace ros;

class MapFrameProcessor
{
   public:
      MapFrameProcessor(ApplicationState& app);

      void init(ApplicationState& app);

      void generateSegmentation(MapFrame frame, vector<shared_ptr<PlanarRegion>>& planarRegionList);

      void dfs(uint16_t x, uint16_t y, uint8_t component, int& num, cv::Mat& debug, shared_ptr<PlanarRegion> planarRegion);

      void boundary_dfs(int x, int y, int regionId, int component, int& num, cv::Mat& debug, shared_ptr<RegionRing> regionRing);

      void findBoundaryAndHoles(vector<shared_ptr<PlanarRegion>>& planarRegionList);

      void printPatchGraph();

      void displayDebugger(int delay);

      void growRegionBoundary(vector<shared_ptr<PlanarRegion>>& regions);

      int adx[8] = {-1, 0, 1, 1, 1, 0, -1, -1};
      int ady[8] = {-1, -1, -1, 0, 1, 1, 1, 0};

   public:
      MapFrame frame;
      cv::Mat debug;
      Eigen::MatrixXi visited;
      Eigen::MatrixXi boundary;
      Eigen::MatrixXi region;
      ApplicationState& app;
      Eigen::Vector2i connect[8] = {Eigen::Vector2i(-1,-1), Eigen::Vector2i(1,0),Eigen::Vector2i(2,0),Eigen::Vector2i(0,1),Eigen::Vector2i(2,1),Eigen::Vector2i(0,2),Eigen::Vector2i(1,2), Eigen::Vector2i(2,2)};

      int SUB_H = 0;
      int SUB_W = 0;
      int PATCH_HEIGHT = 0;
      int PATCH_WIDTH = 0;
      int INPUT_HEIGHT = 0;
      int INPUT_WIDTH = 0;

};

#endif //SRC_MAPFRAMEPROCESSOR_H
