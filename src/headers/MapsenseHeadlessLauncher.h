//
// Created by quantum on 2/22/21.
//

#ifndef MAPSENSEHEADLESSLAUNCHER_H
#define MAPSENSEHEADLESSLAUNCHER_H

#include "PlanarRegionCalculator.h"
#include "NetworkManager.h"

class MapsenseHeadlessLauncher
{
   public:

      MapsenseHeadlessLauncher(int argc, char **argv);

      void update();

      ApplicationState appState;
      PlanarRegionCalculator *_regionCalculator;
      NetworkManager *_dataReceiver;
};

#endif //MAPSENSEHEADLESSLAUNCHER_H
