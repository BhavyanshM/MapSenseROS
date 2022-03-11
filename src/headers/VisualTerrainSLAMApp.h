//
// Created by quantum on 9/17/21.
//

#ifndef MAP_SENSE_CLAYAPP_H
#define MAP_SENSE_CLAYAPP_H

#include "Core/Application.h"
#include "Core/Clay.h"
#include "NetworkedTerrainSLAMLayer.h"

class VisualTerrainSLAMApp : public Clay::Application
{
   public:
      VisualTerrainSLAMApp(int argc, char** argv);
      ~VisualTerrainSLAMApp() = default;
};

#endif //MAP_SENSE_CLAYAPP_H
