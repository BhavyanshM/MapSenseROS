//
// Created by quantum on 12/24/20.
//

#ifndef SRC_APPLICATIONSTATE_H
#define SRC_APPLICATIONSTATE_H

#include <string>

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
      float MERGE_DISTANCE_THRESHOLD = 0.016;
      float MERGE_ANGULAR_THRESHOLD = 0.82;

      bool FILTER_SELECTED = false;
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
      int KERNEL_SLIDER_LEVEL = 2;
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

      bool SHOW_BOUNDARIES = false;
      bool SHOW_PATCHES = true;
      bool VISUAL_DEBUG = false;
      bool SHOW_INPUT_COLOR = false;
      bool SHOW_INPUT_DEPTH = false;
      bool SHOW_FILTERED_DEPTH = false;
      bool SHOW_REGION_COMPONENTS = false;
      bool SHOW_STEREO_LEFT = false;
      bool SHOW_STEREO_RIGHT = false;

      bool SHOW_GRAPH = false;
      bool ROS_ENABLED = true;
      bool SHOW_REGION_EDGES = false;

      bool STEREO_DRIVER = false;
      bool DEPTH_ALIGNED = false;
      bool EARLY_GAUSSIAN_BLUR = true;
      string TOPIC_CAMERA_NAME = "camera";

      int GAUSSIAN_SIZE = 3;
      float GAUSSIAN_SIGMA = 20;

      /*  VISUALIZATION-ONLY */
      float DISPLAY_WINDOW_SIZE = 1.0f;
      float DEPTH_BRIGHTNESS = 40;
      float DEPTH_DISPLAY_OFFSET = 100;

      bool EXPORT_REGIONS = false;
      bool GENERATE_REGIONS = true;

};

#endif //SRC_APPLICATIONSTATE_H
