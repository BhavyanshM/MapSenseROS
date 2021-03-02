//
// Created by quantum on 2/16/21.
//

#ifndef PLOTTER3D_PY_IMGUILAYOUT_H
#define PLOTTER3D_PY_IMGUILAYOUT_H

#include <imgui.h>
#include <ApplicationState.h>
#include "AppUtils.h"
#include <Magnum/GL/Renderer.h>
#include <Magnum/Math/Color.h>

using namespace Magnum;
using namespace Math::Literals;
using namespace std;

class ImGuiLayout
{
   public:
      static void getImGuiROSLayout(ApplicationState& appState);

      static void getImGuiParamsLayout(ApplicationState& appState);

      static void getImGui2DLayout(ApplicationState& appState, uint8_t& displayItem);

      static void getImGui3DLayout(ApplicationState& appState, Color4& color);
};

#endif //PLOTTER3D_PY_IMGUILAYOUT_H
