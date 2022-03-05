//
// Created by quantum on 12/24/20.
//

#ifndef SRC_APPLICATIONSTATE_H
#define SRC_APPLICATIONSTATE_H

#include "MapsenseHeaders.h"

using namespace std;

class ApplicationState
{
   public:
      const string& getDepthFile() const;

      void setDepthFile(const string& depthFile);

      const string& getColorFile() const;

      void setColorFile(const string& colorFile);

      void update();

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

      int INPUT_HEIGHT = 64;
      int INPUT_WIDTH = 1024;
      int KERNEL_SLIDER_LEVEL = 2;
      int PATCH_HEIGHT = KERNEL_SLIDER_LEVEL;
      int PATCH_WIDTH = KERNEL_SLIDER_LEVEL;
      int SUB_H = INPUT_HEIGHT / PATCH_HEIGHT;
      int SUB_W = INPUT_WIDTH / PATCH_WIDTH;

      int FILTER_KERNEL_SIZE = 4;
      int FILTER_SUB_H = INPUT_HEIGHT / FILTER_KERNEL_SIZE;
      int FILTER_SUB_W = INPUT_WIDTH / FILTER_KERNEL_SIZE;

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

      bool SHOW_GRAPH = true;
      bool ROS_ENABLED = true;
      bool SHOW_REGION_EDGES = false;

      bool STEREO_DRIVER = false;
      bool DEPTH_ALIGNED = false;
      bool EARLY_GAUSSIAN_BLUR = true;

      bool ICP_ODOMETRY_ENABLED = false;
      bool STEREO_ODOMETRY_ENABLED = false;
      bool PLANAR_REGIONS_ENABLED = false;
      bool SLAM_ENABLED = false;
      bool DATASET_ENABLED = false;

      string TOPIC_CAMERA_NAME = "chest_l515";

      int GAUSSIAN_SIZE = 4;
      int  GAUSSIAN_SIGMA = 20;

      /*  VISUALIZATION-ONLY */
      float DISPLAY_WINDOW_SIZE = 1.5f;
      float DEPTH_BRIGHTNESS = 40;
      float DEPTH_DISPLAY_OFFSET = 100;

      bool EXPORT_REGIONS = false;
      bool GENERATE_REGIONS = true;

      float REGION_GROWTH_FACTOR = 0.01;

      /* Stereo Matching Parameters */
      int STEREO_NUM_DISPARITIES = 1;
      int STEREO_BLOCK_SIZE = 2;
      int STEREO_PRE_FILTER_SIZE = 2;
      int STEREO_PRE_FILTER_TYPE = 1;
      int STEREO_PRE_FILTER_CAP = 31;
      int STEREO_MIN_DISPARITY = 0;
      int STEREO_TEXTURE_THRESHOLD = 10;
      int STEREO_UNIQUENESS_RATIO = 15;
      int STEREO_SPECKLE_RANGE = 0;
      int STEREO_SPECKLE_WINDOW_SIZE = 0;
      int STEREO_DISP_12_MAX_DIFF = -1;

      string OUSTER_POINTS = "/os_cloud_node/points";
      string ZED_LEFT_IMAGE_RAW = "/zed/color/left/image_raw";
      string ZED_RIGHT_IMAGE_RAW = "/zed/color/right/image_raw";

      string KITTI_LEFT_IMG_RECT = "/kitti/left/image_rect/compressed";
      string KITTI_RIGHT_IMG_RECT = "/kitti/right/image_rect/compressed";
      string KITTI_LIDAR_POINTS = "/kitti/lidar/points";

      string L515_COLOR = "/camera/color/image_raw/compressed";
      string L515_DEPTH = "/chest_l515/depth/image_rect_raw";
      string L515_DEPTH_INFO = "/chest_l515/depth/camera_info";

      string L515_ALIGNED_DEPTH = "/camera/aligned_depth_to_color/image_raw";
      string L515_ALIGNED_DEPTH_INFO = "/camera/aligned_depth_to_color/camera_info";

      string BLACKFLY_RIGHT_RAW = "/blackfly/right/image_color";


};

#endif //SRC_APPLICATIONSTATE_H
