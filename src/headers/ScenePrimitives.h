//
// Created by quantum on 12/24/20.
//

#ifndef SRC_SCENEPRIMITIVES_H
#define SRC_SCENEPRIMITIVES_H

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
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Math/Quaternion.h>
#include <Magnum/MeshTools/Interleave.h>
#include <Magnum/GL/Renderer.h>

#include <iostream>
#include <imgui.h>

#include "MeshGenerator.h"
#include "PlanarRegionCalculator.h"
#include "ApplicationState.h"
#include "sys/resource.h"

using namespace Magnum;
using namespace Math::Literals;


typedef SceneGraph::Object<SceneGraph::MatrixTransformation3D> Object3D;
typedef SceneGraph::Scene<SceneGraph::MatrixTransformation3D> Scene3D;

class MyApplication : public Platform::Application {
public:
    explicit MyApplication(const Arguments &arguments);

private:
    void drawEvent() override;
    void tickEvent() override;

    void mousePressEvent(MouseEvent &event) override;
    void mouseReleaseEvent(MouseEvent& event) override;
    void mouseMoveEvent(MouseMoveEvent &event) override;
    void mouseScrollEvent(MouseScrollEvent &event) override;

    void viewportEvent(ViewportEvent& event) override;
    void generate_patches();
    void draw_patches();
    void draw_regions();

    ApplicationState appState;
    ImGuiIntegration::Context _imgui{NoCreate};
    Color4 _clearColor = 0x72909aff_rgbaf;
    Float _floatValue = 0.0f;

    uint8_t _displayItem = -1;

    Scene3D _scene;
    Object3D *_camGrandParent;
    Object3D *_camParent;
    Object3D *_camObject;
    Object3D *_camOriginCube;
    Object3D *_sensor;
    Object3D *_sensorAxes;
    SceneGraph::Camera3D *_camera;

    vector<Object3D*> planePatches;
    vector<Object3D*> regionEdges;

    SceneGraph::DrawableGroup3D _drawables;

    PlanarRegionCalculator* _regionCalculator;
    NetworkManager* _dataReceiver;
    int count = 0;
};

class RedCubeDrawable : public SceneGraph::Drawable3D {
public:
    explicit RedCubeDrawable(Object3D &object, SceneGraph::DrawableGroup3D *group, Trade::MeshData meshData, Vector3 color) :
            SceneGraph::Drawable3D{object, group} {
        _color = color;
        _mesh = MeshTools::compile(meshData);
    }


    explicit RedCubeDrawable(Object3D &object, SceneGraph::DrawableGroup3D *group, GL::Buffer& vertexBuffer, Vector3 color) :
            SceneGraph::Drawable3D{object, group} {
        _color = color;
        _mesh.setPrimitive(MeshPrimitive::TriangleFan)
                .addVertexBuffer(vertexBuffer, 0, Position{})
                .setCount(vertexBuffer.size());
    }

private:
    GL::Mesh _mesh;
    Shaders::Phong _shader;
    Vector3 _color;

    void draw(const Matrix4 &transformationMatrix, SceneGraph::Camera3D &camera) override {

        // _mesh.setPrimitive(GL::MeshPrimitive::Points);
        _shader.setDiffuseColor(0xa5c9ea_rgbf)
                .setLightColor(Color3{1.0f})
                .setLightPosition({0.0f, 2.0f, 0.0f})
                .setAmbientColor(_color)
                .setTransformationMatrix(transformationMatrix)
                .setNormalMatrix(transformationMatrix.normalMatrix())
                .setProjectionMatrix(camera.projectionMatrix())
                .draw(_mesh);
    }
};

class PointCloudDrawable : public SceneGraph::Drawable3D {
public:
    explicit PointCloudDrawable(Object3D &object, SceneGraph::DrawableGroup3D *group, GL::Buffer& vertexBuffer, Vector3 color) :
            SceneGraph::Drawable3D{object, group} {
        _color = color;
        _mesh.setPrimitive(MeshPrimitive::Points)
                .addVertexBuffer(vertexBuffer, 0, Position{})
                .setCount(vertexBuffer.size());
    }

private:
    GL::Mesh _mesh;
    Shaders::Phong _shader;
    Vector3 _color;

    void draw(const Matrix4 &transformationMatrix, SceneGraph::Camera3D &camera) override {

         _mesh.setPrimitive(GL::MeshPrimitive::Points);
        _shader.setDiffuseColor(0xa5c9ea_rgbf)
                .setLightColor(Color3{1.0f})
                .setLightPosition({0.0f, 2.0f, 0.0f})
                .setAmbientColor(_color)
                .setTransformationMatrix(transformationMatrix)
                .setNormalMatrix(transformationMatrix.normalMatrix())
                .setProjectionMatrix(camera.projectionMatrix())
                .draw(_mesh);
    }
};



#endif //SRC_SCENEPRIMITIVES_H
