//
// Created by quantum on 2/20/21.
//

#ifndef MAGNUMAPPLICATION_H
#define MAGNUMAPPLICATION_H

#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/Line.h>
#include <Magnum/Primitives/Axis.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/Shaders/PhongGL.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/MeshTools/Interleave.h>
#include <Magnum/GL/Renderer.h>

#include <PlanarRegionMapHandler.h>
#include "MeshGenerator.h"

using namespace Magnum::Math::Literals;

typedef Magnum::SceneGraph::Object<Magnum::SceneGraph::MatrixTransformation3D> Object3D;
typedef Magnum::SceneGraph::Scene<Magnum::SceneGraph::MatrixTransformation3D> Scene3D;

   class MagnumApplication : public Magnum::Platform::Application
{
   public:
      explicit MagnumApplication(const Arguments& arguments);

   protected:
      void drawEvent() override;

      virtual void draw() = 0;

      void viewportEvent(ViewportEvent& event) override;

      void mouseMoveEvent(MouseMoveEvent& event) override;

      void mouseScrollEvent(MouseScrollEvent& event) override;

      Scene3D _scene;
      Object3D *_camGrandParent;
      Object3D *_camParent;
      Object3D *_camObject;
      Object3D *_camOriginCube;
      Object3D *_world;
      Object3D *_sensorAxes;
      Magnum::SceneGraph::Camera3D *_camera;

      Magnum::SceneGraph::DrawableGroup3D _drawables;
};





#endif //MAGNUMAPPLICATION_H
