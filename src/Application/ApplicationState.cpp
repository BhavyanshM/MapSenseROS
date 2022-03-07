//
// Created by quantum on 12/24/20.
//

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include "ApplicationState.h"

ApplicationState::ApplicationState()
{
   std::string path = ros::package::getPath("map_sense") + "/Extras/Config/MapsenseParameters.txt";

   std::ifstream infile{ path };

   std::string str, name, value;
   for (int i = 0; i<100; i++)
   {
      getline(infile, str);
      std::vector<std::string> strs;
      boost::split(strs,str,boost::is_any_of(":"));
      name = strs[0];
      if(strs.size() > 1)
      {
         value = strs[1];
         value.erase(std::remove_if(value.begin(), value.end(), ::isspace), value.end());
      }


      printf("NAME: %s, VALUE: %s\n", name.c_str(), value.c_str());
      if(str == "# end") break;


      if(name == "REGION_MIN_PATCHES") {REGION_MIN_PATCHES = stoi(value);}
      if(name == "REGION_BOUNDARY_DIFF") {REGION_BOUNDARY_DIFF = stoi(value);}
      if(name == "REGION_MODE") {REGION_MODE = stoi(value);}
      if(name == "INPUT_HEIGHT") {INPUT_HEIGHT = stoi(value);}
      if(name == "INPUT_WIDTH") {INPUT_WIDTH = stoi(value);}
      if(name == "KERNEL_SLIDER_LEVEL") {KERNEL_SLIDER_LEVEL = stoi(value);}
      if(name == "FILTER_KERNEL_SIZE") {FILTER_KERNEL_SIZE = stoi(value);}
      if(name == "DIVISION_FACTOR") {DIVISION_FACTOR = stoi(value);}
      if(name == "NUM_SKIP_EDGES") {NUM_SKIP_EDGES = stoi(value);}
      if(name == "VISUAL_DEBUG_DELAY") {VISUAL_DEBUG_DELAY = stoi(value);}
      if(name == "GAUSSIAN_SIZE") {GAUSSIAN_SIZE = stoi(value);}
      if(name == "GAUSSIAN_SIGMA") {GAUSSIAN_SIGMA = stoi(value);}
      if(name == "STEREO_NUM_DISPARITIES") {STEREO_NUM_DISPARITIES = stoi(value);}
      if(name == "STEREO_BLOCK_SIZE") {STEREO_BLOCK_SIZE = stoi(value);}
      if(name == "STEREO_PRE_FILTER_SIZE") {STEREO_PRE_FILTER_SIZE = stoi(value);}
      if(name == "HASH_THREAD_NUM") {HASH_THREAD_NUM = stoi(value);}
      if(name == "PATCH_HEIGHT") {PATCH_HEIGHT = stoi(value);}
      if(name == "PATCH_WIDTH") {PATCH_WIDTH = stoi(value);}

      if(name == "FILTER_DISPARITY_THRESHOLD") {FILTER_DISPARITY_THRESHOLD = stof(value);}
      if(name == "DEPTH_BRIGHTNESS") {DEPTH_BRIGHTNESS = stof(value);}
      if(name == "DEPTH_DISPLAY_OFFSET") {DEPTH_DISPLAY_OFFSET = stof(value);}
      if(name == "DEPTH_FX") {DEPTH_FX = stof(value);}
      if(name == "DEPTH_FY") {DEPTH_FY = stof(value);}
      if(name == "DEPTH_CX") {DEPTH_CX = stof(value);}
      if(name == "DEPTH_CY") {DEPTH_CY = stof(value);}
      if(name == "MERGE_DISTANCE_THRESHOLD") {MERGE_DISTANCE_THRESHOLD = stof(value);}
      if(name == "MERGE_ANGULAR_THRESHOLD") {MERGE_ANGULAR_THRESHOLD = stof(value);}
      if(name == "MAGNUM_PATCH_SCALE") {MAGNUM_PATCH_SCALE = stof(value);}
      if(name == "DISPLAY_WINDOW_SIZE") {DISPLAY_WINDOW_SIZE = stof(value);}
      if(name == "REGION_GROWTH_FACTOR") {REGION_GROWTH_FACTOR = stof(value);}

      if(name == "FILTER_SELECTED") {FILTER_SELECTED = (value == "true");}
      if(name == "SHOW_BOUNDARIES") {SHOW_BOUNDARIES = (value == "true");}
      if(name == "SHOW_PATCHES") {SHOW_PATCHES = (value == "true");}
      if(name == "VISUAL_DEBUG") {VISUAL_DEBUG = (value == "true");}
      if(name == "SHOW_INPUT_COLOR") {SHOW_INPUT_COLOR = (value == "true");}
      if(name == "SHOW_INPUT_DEPTH") {SHOW_INPUT_DEPTH = (value == "true");}
      if(name == "SHOW_FILTERED_DEPTH") {SHOW_FILTERED_DEPTH = (value == "true");}
      if(name == "SHOW_REGION_COMPONENTS") {SHOW_REGION_COMPONENTS = (value == "true");}
      if(name == "SHOW_STEREO_LEFT") {SHOW_STEREO_LEFT = (value == "true");}
      if(name == "SHOW_STEREO_RIGHT") {SHOW_STEREO_RIGHT = (value == "true");}
      if(name == "SHOW_GRAPH") {SHOW_GRAPH = (value == "true");}
      if(name == "ROS_ENABLED") {ROS_ENABLED = (value == "true");}
      if(name == "SHOW_REGION_EDGES") {SHOW_REGION_EDGES = (value == "true");}
      if(name == "STEREO_DRIVER") {STEREO_DRIVER = (value == "true");}
      if(name == "DEPTH_ALIGNED") {DEPTH_ALIGNED = (value == "true");}
      if(name == "EARLY_GAUSSIAN_BLUR") {EARLY_GAUSSIAN_BLUR = (value == "true");}
      if(name == "ICP_ODOMETRY_ENABLED") {ICP_ODOMETRY_ENABLED = (value == "true");}
      if(name == "STEREO_ODOMETRY_ENABLED") {STEREO_ODOMETRY_ENABLED = (value == "true");}
      if(name == "PLANAR_REGIONS_ENABLED") {PLANAR_REGIONS_ENABLED = (value == "true");}
      if(name == "SLAM_ENABLED") {SLAM_ENABLED = (value == "true");}
      if(name == "DATASET_ENABLED") {DATASET_ENABLED = (value == "true");}
      if(name == "EXPORT_REGIONS") {EXPORT_REGIONS = (value == "true");}
      if(name == "GENERATE_REGIONS") {GENERATE_REGIONS = (value == "true");}
   }

   SUB_H = (int) INPUT_HEIGHT / PATCH_HEIGHT;
   SUB_W = (int) INPUT_WIDTH / PATCH_WIDTH;
}

void ApplicationState::update()
{
//   if (INPUT_HEIGHT > 0 && INPUT_WIDTH > 0)
//   {
//      if ((INPUT_HEIGHT % KERNEL_SLIDER_LEVEL == 0) && (INPUT_WIDTH % KERNEL_SLIDER_LEVEL == 0))
//      {
//         PATCH_HEIGHT = KERNEL_SLIDER_LEVEL;
//         PATCH_WIDTH = KERNEL_SLIDER_LEVEL;
//         SUB_H = (int) INPUT_HEIGHT / PATCH_HEIGHT;
//         SUB_W = (int) INPUT_WIDTH / PATCH_WIDTH;
//      }
//      if ((INPUT_HEIGHT % FILTER_KERNEL_SIZE == 0) && (INPUT_WIDTH % FILTER_KERNEL_SIZE == 0))
//      {
//         FILTER_SUB_H = (int) INPUT_HEIGHT / FILTER_KERNEL_SIZE;
//         FILTER_SUB_W = (int) INPUT_WIDTH / FILTER_KERNEL_SIZE;
//      }
//   }
}

