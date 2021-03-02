//
// Created by quantum on 2/22/21.
//

#include "MapsenseHeadlessLauncher.h"

MapsenseHeadlessLauncher::MapsenseHeadlessLauncher(int argc, char **argv)
{
   _dataReceiver = new NetworkManager();
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
   while (mapsense.appState.ROS_ENABLED)
   {
      mapsense.update();
   }
}