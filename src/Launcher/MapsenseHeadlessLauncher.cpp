//
// Created by quantum on 2/22/21.
//

#include "MapsenseHeadlessLauncher.h"

MapsenseHeadlessLauncher::MapsenseHeadlessLauncher(int argc, char **argv)
{
   _openCLManager = new OpenCLManager();
   _networkManager = new NetworkManager(this->appState, &appUtils);
   _networkManager->init_ros_node(argc, argv, this->appState);
   _regionCalculator = new PlanarRegionCalculator(argc, argv, _networkManager, appState);
   _regionCalculator->setOpenCLManager(_openCLManager);
}

void MapsenseHeadlessLauncher::update()
{
   _networkManager->spin_ros_node();
   _networkManager->acceptMapsenseConfiguration(appState);
   _networkManager->receiverUpdate(appState);
   if (_networkManager->paramsAvailable)
   {
      _networkManager->paramsAvailable = false;
      appState.MERGE_DISTANCE_THRESHOLD = _networkManager->paramsMessage.mergeDistanceThreshold;
      appState.MERGE_ANGULAR_THRESHOLD = _networkManager->paramsMessage.mergeAngularThreshold;
   }

   _regionCalculator->generateRegionsFromDepth(appState);
   _regionCalculator->publish();
}

int main(int argc, char **argv)
{
   MapsenseHeadlessLauncher mapsense(argc, argv);

   std::vector<std::string> args(argv, argv + argc);

   for (int i = 0; i < argc; i++)
   {
      if (args[i] == "--export")
      {
         printf("Setting EXPORT_REGIONS: true\n");
         mapsense.appState.EXPORT_REGIONS = true;
      }
      if (args[i] == "--kernel-level")
      {
         printf("Setting KERNEL_LEVEL_SLIDER: %d\n", stoi(args[i + 1]));
         mapsense.appState.KERNEL_SLIDER_LEVEL = stoi(args[i + 1]);
      }
      if (args[i] == "--depth-aligned")
      {
         printf("Setting DEPTH_ALIGNED: true\n");
         mapsense.appState.DEPTH_ALIGNED = true;
      }
   }

   while (mapsense.appState.ROS_ENABLED)
   {
      mapsense.update();
   }
}