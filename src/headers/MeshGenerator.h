#ifndef SRC_MESHGENERATOR_H
#define SRC_MESHGENERATOR_H

#include <iostream>
#include <opencv2/core.hpp>
#include "opencv2/imgproc.hpp"

#include "Magnum/Math/Functions.h"
#include <Magnum/GL/Buffer.h>
#include <Magnum/Trade/MeshData.h>
#include "Magnum/Math/Color.h"
#include "Magnum/Mesh.h"
#include <Magnum/Math/Quaternion.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/MeshTools/Interleave.h>
#include <Magnum/MeshTools/Compile.h>

#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Primitives/Plane.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/Line.h>
#include <Magnum/Primitives/Circle.h>
#include <Magnum/Primitives/Circle.h>
#include <Magnum/Primitives/Axis.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/Shaders/PhongGL.h>

#include <Magnum/GL/Mesh.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Math/Quaternion.h>

#include <Corrade/Containers/ArrayViewStl.h>

#include "PlanarRegion.h"

using namespace cv;
using namespace std;
using namespace Magnum::Math::Literals;


typedef Magnum::GL::Attribute<0, Magnum::Vector3> Position;
typedef Magnum::SceneGraph::Object<Magnum::SceneGraph::MatrixTransformation3D> Object3D;
typedef Magnum::SceneGraph::Scene<Magnum::SceneGraph::MatrixTransformation3D> Scene3D;



class MeshGenerator
{
   private:
      const int SKIP_EDGES = 1;

      Magnum::SceneGraph::DrawableGroup3D* drawables;

   public:

      void generateRegionLineMesh(vector<shared_ptr<PlanarRegion>> planarRegionList, vector<Object3D *>& regionEdges, int color, Object3D* parent, bool erase = false);

      void generateMatchLineMesh(vector<pair<int,int>> matches, vector<shared_ptr<PlanarRegion>> regions, vector<shared_ptr<PlanarRegion>> latestRegions, vector<Object3D *>& edges, Object3D* parent);

      void generatePoseMesh(vector<RigidBodyTransform> poses, vector<Object3D*>& edges, Object3D* parent);

      explicit MeshGenerator(Magnum::SceneGraph::DrawableGroup3D* drawables);

      static void clearMesh(vector<Object3D *>& objects);
};

class RedCubeDrawable : public Magnum::SceneGraph::Drawable3D
{
   public:
      explicit RedCubeDrawable(Object3D& object, Magnum::SceneGraph::DrawableGroup3D *group, Magnum::Trade::MeshData meshData, Magnum::Vector3 color) : Magnum::SceneGraph::Drawable3D{object,
                                                                                                                                                                                       group}
      {
         _color = color;
         _mesh = Magnum::MeshTools::compile(meshData);
      }

      typedef Magnum::GL::Attribute<0, Magnum::Vector3> Position;

      explicit RedCubeDrawable(Object3D& object, Magnum::SceneGraph::DrawableGroup3D *group, Magnum::GL::Buffer& vertexBuffer, Magnum::Vector3 color) : Magnum::SceneGraph::Drawable3D{object,
                                                                                                                                                                                       group}
      {
         _color = color;
         _mesh.setPrimitive(Magnum::MeshPrimitive::TriangleFan).addVertexBuffer(vertexBuffer, 0, Position{}).setCount(vertexBuffer.size());
      }

   private:
      Magnum::GL::Mesh _mesh;
      Magnum::Shaders::Phong _shader;
      Magnum::Vector3 _color;

      void draw(const Magnum::Matrix4& transformationMatrix, Magnum::SceneGraph::Camera3D& camera) override
      {

         // _mesh.setPrimitive(GL::MeshPrimitive::Points);
         _shader.setDiffuseColor(0xa5c9ea_rgbf).setLightColor(Magnum::Color3{1.0f}).setLightPosition({0.0f, 2.0f, 0.0f}).setAmbientColor(
               _color).setTransformationMatrix(transformationMatrix).setNormalMatrix(transformationMatrix.normalMatrix()).setProjectionMatrix(
               camera.projectionMatrix()).draw(_mesh);
      }
};

class PointCloudDrawable : public Magnum::SceneGraph::Drawable3D
{
   public:
      explicit PointCloudDrawable(Object3D& object, Magnum::SceneGraph::DrawableGroup3D *group, Magnum::GL::Buffer& vertexBuffer, Magnum::Vector3 color) : Magnum::SceneGraph::Drawable3D{
            object, group}
      {
         _color = color;
         _mesh.setPrimitive(Magnum::MeshPrimitive::Points).addVertexBuffer(vertexBuffer, 0, Position{}).setCount(vertexBuffer.size());
      }

   private:
      Magnum::GL::Mesh _mesh;
      Magnum::Shaders::Phong _shader;
      Magnum::Vector3 _color;

      void draw(const Magnum::Matrix4& transformationMatrix, Magnum::SceneGraph::Camera3D& camera)
      {

         _mesh.setPrimitive(Magnum::GL::MeshPrimitive::Points);
         _shader.setDiffuseColor(0xa5c9ea_rgbf).setLightColor(Magnum::Color3{1.0f}).setLightPosition({0.0f, 2.0f, 0.0f}).setAmbientColor(
               _color).setTransformationMatrix(transformationMatrix).setNormalMatrix(transformationMatrix.normalMatrix()).setProjectionMatrix(
               camera.projectionMatrix()).draw(_mesh);
      }
};

#endif //SRC_MESHGENERATOR_H
