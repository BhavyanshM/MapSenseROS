//
// Created by quantum on 2/16/21.
//

#ifndef IMGUILAYOUT_H
#define IMGUILAYOUT_H

#include <imgui.h>
#include <ApplicationState.h>
#include "AppUtils.h"
#include <Magnum/GL/Renderer.h>
#include <Magnum/Math/Color.h>

using namespace Magnum::Math::Literals;
using namespace std;

class ImGuiLayout
{
   public:
      static void getImGuiROSLayout(ApplicationState& appState);

      static void getImGuiParamsLayout(ApplicationState& appState);

      static void getImGui2DLayout(ApplicationState& appState);

      static void getImGui3DLayout(ApplicationState& appState, Magnum::Color4& color);
};

#endif //IMGUILAYOUT_H
