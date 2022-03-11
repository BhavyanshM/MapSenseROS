//
// Created by quantum on 2/16/21.
//

#ifndef IMGUILAYOUT_H
#define IMGUILAYOUT_H

#include <imgui.h>
#include <ApplicationState.h>
#include "AppUtils.h"

class ImGuiLayout
{
   public:
      static void getImGuiAppLayout(ApplicationState& appState);

      static void getImGui2DLayout(ApplicationState& appState);

      static void getImGui3DLayout(ApplicationState& appState);
};

#endif //IMGUILAYOUT_H
