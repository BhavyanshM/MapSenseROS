//
// Created by quantum on 12/24/20.
//

#ifndef SRC_APPLICATIONSTATE_H
#define SRC_APPLICATIONSTATE_H

#include <string>

#define SHOW_INPUT_DEPTH 1
#define SHOW_FILTERED_DEPTH 2
#define SHOW_REGION_COMPONENTS 0
#define SHOW_INPUT_COLOR 3

using namespace std;

class ApplicationState
{
   public:
      const string& getDepthFile() const;

      void setDepthFile(const string& depthFile);

      const string& getColorFile() const;

      void setColorFile(const string& colorFile);

      void update();

   private:
      string depthFile = "/data/Depth_L515.png";
      string colorFile = "/data/Color_L515.png";

   public:
      float MERGE_DISTANCE_THRESHOLD = 0.035;
      float MERGE_ANGULAR_THRESHOLD = 0.7;

      bool FILTER_SELECTED = true;
      float FILTER_DISPARITY_THRESHOLD = 2000;
      float MAGNUM_PATCH_SCALE = 0.007;

      int REGION_MIN_PATCHES = 20;
      int REGION_BOUNDARY_DIFF = 20;

      /*
       * NOTE: The following parameters should meet these requirements.
       * a) InputHeight should be divisible by (KernelResLevel * AspectRatioHeight)
       * b) InputWidth should be divisible by (KernelResLevel * AspectRatioWidth)
       * */

      int INPUT_HEIGHT = 0;
      int INPUT_WIDTH = 0;
      int KERNEL_SLIDER_LEVEL = 4;
      int PATCH_HEIGHT = KERNEL_SLIDER_LEVEL;
      int PATCH_WIDTH = KERNEL_SLIDER_LEVEL;
      int SUB_H = (int) INPUT_HEIGHT / PATCH_HEIGHT;
      int SUB_W = (int) INPUT_WIDTH / PATCH_WIDTH;

      int FILTER_KERNEL_SIZE = 4;
      int FILTER_SUB_H = (int) INPUT_HEIGHT / FILTER_KERNEL_SIZE;
      int FILTER_SUB_W = (int) INPUT_WIDTH / FILTER_KERNEL_SIZE;

      //    float DEPTH_FX = 459.97;
      //    float DEPTH_FY = 459.80;
      //    float DEPTH_CX = 341.84;
      //    float DEPTH_CY = 249.17;

      int DIVISION_FACTOR = 1; // To simulate lower resolution by a factor.

      float DEPTH_FX = 0;
      float DEPTH_FY = 0;
      float DEPTH_CX = 0;
      float DEPTH_CY = 0;

      int NUM_SKIP_EDGES = 8;
      int VISUAL_DEBUG_DELAY = 10;

      bool SHOW_BOUNDARIES = true;
      bool SHOW_PATCHES = true;
      bool VISUAL_DEBUG = false;

      bool SHOW_GRAPH = true;
      bool ROS_ENABLED = true;
      bool SHOW_REGION_EDGES = false;

      /*  VISUALIZATION-ONLY */
      float DEPTH_BRIGHTNESS = 40;
      float DEPTH_DISPLAY_OFFSET = 100;

      bool EXPORT_REGIONS = false;

};

#endif //SRC_APPLICATIONSTATE_H
