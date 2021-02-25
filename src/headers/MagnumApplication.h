//
// Created by quantum on 2/20/21.
//

#ifndef SLAMAPPLICATION_H
#define SLAMAPPLICATION_H

#include <PlanarRegionMapHandler.h>

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
#include <Magnum/Shaders/Phong.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/MeshTools/Interleave.h>
#include <Magnum/GL/Renderer.h>

using namespace Magnum;
using namespace Math::Literals;

typedef SceneGraph::Object<SceneGraph::MatrixTransformation3D> Object3D;
typedef SceneGraph::Scene<SceneGraph::MatrixTransformation3D> Scene3D;

class MagnumApplication : public Platform::Application
{
   public:
      explicit MagnumApplication(const Arguments& arguments);

   protected:
      void drawEvent() override;

      virtual void draw() = 0;

      void viewportEvent(ViewportEvent& event);

      void mouseMoveEvent(MouseMoveEvent& event);

      void mouseScrollEvent(MouseScrollEvent& event);

      Scene3D _scene;
      Object3D *_camGrandParent;
      Object3D *_camParent;
      Object3D *_camObject;
      Object3D *_camOriginCube;
      Object3D *_sensor;
      Object3D *_sensorAxes;
      SceneGraph::Camera3D *_camera;

      SceneGraph::DrawableGroup3D _drawables;
};

class RedCubeDrawable : public SceneGraph::Drawable3D
{
   public:
      explicit RedCubeDrawable(Object3D& object, SceneGraph::DrawableGroup3D *group, Trade::MeshData meshData, Vector3 color) : SceneGraph::Drawable3D{object,
                                                                                                                                                       group}
      {
         _color = color;
         _mesh = MeshTools::compile(meshData);
      }

      typedef GL::Attribute<0, Vector3> Position;

      explicit RedCubeDrawable(Object3D& object, SceneGraph::DrawableGroup3D *group, GL::Buffer& vertexBuffer, Vector3 color) : SceneGraph::Drawable3D{object,
                                                                                                                                                       group}
      {
         _color = color;
         _mesh.setPrimitive(MeshPrimitive::TriangleFan).addVertexBuffer(vertexBuffer, 0, Position{}).setCount(vertexBuffer.size());
      }

   private:
      GL::Mesh _mesh;
      Shaders::Phong _shader;
      Vector3 _color;

      void draw(const Matrix4& transformationMatrix, SceneGraph::Camera3D& camera) override
      {

         // _mesh.setPrimitive(GL::MeshPrimitive::Points);
         _shader.setDiffuseColor(0xa5c9ea_rgbf).setLightColor(Color3{1.0f}).setLightPosition({0.0f, 2.0f, 0.0f}).setAmbientColor(
               _color).setTransformationMatrix(transformationMatrix).setNormalMatrix(transformationMatrix.normalMatrix()).setProjectionMatrix(
               camera.projectionMatrix()).draw(_mesh);
      }
};

#endif //SLAMAPPLICATION_H
