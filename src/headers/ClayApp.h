//
// Created by quantum on 9/17/21.
//

#ifndef MAP_SENSE_CLAYAPP_H
#define MAP_SENSE_CLAYAPP_H

#include "Core/Application.h"
#include "Core/Clay.h"
#include "MapsenseLayer.h"

class ClayApp : public Clay::Application
{
   public:
      ClayApp(int argc, char** argv);
      ~ClayApp() = default;
};

#endif //MAP_SENSE_CLAYAPP_H
