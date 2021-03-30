//
// Created by quantum on 2/22/21.
//

#include "MapsenseHeadlessLauncher.h"

MapsenseHeadlessLauncher::MapsenseHeadlessLauncher(int argc, char **argv)
{
   _dataReceiver = new NetworkManager(this->appState);
   _dataReceiver->init_ros_node(argc, argv);
   _regionCalculator = new PlanarRegionCalculator(appState);
   _regionCalculator->initOpenCL(appState);

   namedWindow("DebugOutput", WINDOW_NORMAL);
}

void MapsenseHeadlessLauncher::update()
{
   _dataReceiver->spin_ros_node();
   _dataReceiver->load_next_frame(_regionCalculator->inputDepth, _regionCalculator->inputColor, appState);
   if (_dataReceiver->depthCamInfoSet)
   {
      _regionCalculator->generateRegions(_dataReceiver, appState);
   }
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
   }

   while (mapsense.appState.ROS_ENABLED)
   {
      mapsense.update();
   }
}