#ifndef SRC_MAPFRAMEPROCESSOR_H
#define SRC_MAPFRAMEPROCESSOR_H

#include "ros/ros.h"
#include "MapFrame.h"
#include "PlanarRegion.h"
#include <opencv2/highgui.hpp>
#include "ApplicationState.h"
#include <algorithm>

using namespace ros;

class MapFrameProcessor
{
   public:

      MapFrame frame;
      Mat debug;
      MatrixXi visited;
      MatrixXi boundary;
      MatrixXi region;
      ApplicationState app;
      Vector2i connect[8] = {Vector2i(-1,-1), Vector2i(1,0),Vector2i(2,0),Vector2i(0,1),Vector2i(2,1),Vector2i(0,2),Vector2i(1,2), Vector2i(2,2)};

      void init(ApplicationState& app);

      void generateSegmentation(MapFrame frame, vector<shared_ptr<PlanarRegion>>& planarRegionList);

      void dfs(uint16_t x, uint16_t y, uint8_t component, int& num, Mat& debug, shared_ptr<PlanarRegion> planarRegion);

      void boundary_dfs(int x, int y, int component, int& num, Mat& debug, shared_ptr<RegionRing> regionRing);

      void findBoundaryAndHoles(vector<shared_ptr<PlanarRegion>>& planarRegionList);

      void printPatchGraph();

      void displayDebugger(int delay);

      int adx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
      int ady[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
};

#endif //SRC_MAPFRAMEPROCESSOR_H
