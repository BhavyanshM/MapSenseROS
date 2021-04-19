//
// Created by quantum on 12/24/20.
//

#ifndef MAPSENSE_LAUNCHER_UI_H
#define MAPSENSE_LAUNCHER_UI_H

#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/ImGuiIntegration/Context.h>
#include <Magnum/ImGuiIntegration/Context.hpp>
#include <Magnum/Primitives/Plane.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/Line.h>
#include <Magnum/Primitives/Circle.h>
#include <Magnum/Primitives/Circle.h>
#include <Magnum/Primitives/Axis.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/Shaders/Phong.h>

#include <Magnum/GL/Mesh.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Math/Quaternion.h>

#include <Magnum/GL/Renderer.h>

#include <iostream>
#include <imgui.h>

#include "MeshGenerator.h"
#include "PlanarRegionCalculator.h"
#include "ApplicationState.h"
#include "ImGuiLayout.h"
#include "AppUtils.h"

using namespace Magnum::Math::Literals;

   class MyApplication : public Magnum::Platform::Application
{
   public:
      explicit MyApplication(const Arguments& arguments);

      void init(int argc, char** argv);

      ApplicationState appState;

      AppUtils appUtils;

   private:
      void drawEvent() override;

      void tickEvent() override;

      void exitEvent(ExitEvent& event) override;

      void mousePressEvent(MouseEvent& event) override;

      void mouseReleaseEvent(MouseEvent& event) override;

      void mouseMoveEvent(MouseMoveEvent& event) override;

      void mouseScrollEvent(MouseScrollEvent& event) override;

      void viewportEvent(ViewportEvent& event) override;

      void generate_patches();

      void draw_patches();

      void draw_regions();

      Magnum::ImGuiIntegration::Context _imgui{Magnum::NoCreate};
      Magnum::Color4 _clearColor = 0x72909aff_rgbaf;
      float _floatValue = 0.0f;

      Scene3D _scene;
      Object3D *_camGrandParent;
      Object3D *_camParent;
      Object3D *_camObject;
      Object3D *_camOriginCube;
      Object3D *_sensor;
      Object3D *_sensorAxes;
      Magnum::SceneGraph::Camera3D *_camera;

      vector<Object3D *> planePatches;
      vector<Object3D *> regionEdges;

      Magnum::SceneGraph::DrawableGroup3D _drawables;

      PlanarRegionCalculator *_regionCalculator;
      NetworkManager *_dataReceiver;
      int count, frameId = 0;
};



#endif //MAPSENSE_LAUNCHER_UI_H
