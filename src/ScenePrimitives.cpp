#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>

#include <Magnum/GL/Mesh.h>

#include <Magnum/Primitives/Plane.h>
#include <Magnum/Primitives/Cube.h>
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
#include <iostream>
#include <Magnum/GL/Renderer.h>
#include "PlanarRegionCalculator.h"

using namespace Magnum;

typedef SceneGraph::Object<SceneGraph::MatrixTransformation3D> Object3D;
typedef SceneGraph::Scene<SceneGraph::MatrixTransformation3D> Scene3D;

class MyApplication : public Platform::Application {
public:
    explicit MyApplication(const Arguments &arguments);

private:
    void drawEvent() override;

    void tickEvent() override;

    void mousePressEvent(MouseEvent &event) override;

    void mouseMoveEvent(MouseMoveEvent &event) override;

    void mouseScrollEvent(MouseScrollEvent &event) override;


    Scene3D _scene;
    Object3D *_camGrandParent;
    Object3D *_camParent;
    Object3D *_camObject;
    Object3D *_camOriginCube;
    Object3D *_sensor;
    Object3D *_sensorAxes;
    SceneGraph::Camera3D *_camera;

    SceneGraph::DrawableGroup3D _drawables;

    PlanarRegionCalculator* _regionCalculator;
    int count = 0;
};

class RedCubeDrawable : public SceneGraph::Drawable3D {
public:
    explicit RedCubeDrawable(Object3D &object, SceneGraph::DrawableGroup3D *group, Trade::MeshData meshData, Vector3 color) :
            SceneGraph::Drawable3D{object, group} {
        _color = color;
        _mesh = MeshTools::compile(meshData);
    }

private:
    GL::Mesh _mesh;
    Shaders::Phong _shader;
    Vector3 _color;

    void draw(const Matrix4 &transformationMatrix, SceneGraph::Camera3D &camera) override {
        using namespace Math::Literals;
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

MyApplication::MyApplication(const Arguments &arguments) : Platform::Application{arguments} {
    /* TODO: Instantiate the Information Processor*/
    _regionCalculator = new PlanarRegionCalculator();
    _regionCalculator->init_opencl();
    _regionCalculator->launch_tester();

    /* TODO: Check that the appropriate flags for renderer are set*/
    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    // GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);

    using namespace Math::Literals;

    /* TODO: Configure Camera in Scene Graph*/
    _camGrandParent = new Object3D{&_scene};
    _sensor = new Object3D{&_scene};
    _camParent = new Object3D{_camGrandParent};
    _camObject = new Object3D{_camParent};
    _camera = new SceneGraph::Camera3D(*_camObject);
    _camera->setProjectionMatrix(Matrix4::perspectiveProjection(35.0_degf,1.33f, 0.001f, 100.0f));


    /* TODO: Prepare your objects here and add them to the scene */
    _sensorAxes = new Object3D{_sensor};
    _sensorAxes->scale({0.01,0.01,0.01});
    new RedCubeDrawable{*_sensorAxes, &_drawables, Primitives::axis3D(), {0.5, 0.1f, 0.1f}};

    _camObject->translate({0,0,0.4f});
    _sensor->transformLocal(Matrix4::rotationX(Rad{90.0_degf}));

    _camOriginCube = new Object3D{_camGrandParent};
    _camOriginCube->scale({0.0004f, 0.0004f, 0.0004f});
    new RedCubeDrawable{*_camOriginCube, &_drawables, Primitives::cubeSolid(), {0.2, 0.0f, 0.3f}};

    for(int i = 0; i < _regionCalculator->regionOutput.rows; i++){
        for(int j = 0; j < _regionCalculator->regionOutput.cols; j++){
            uint8_t edges = _regionCalculator->patchData.at<uint8_t>(i, j);
            if (edges == 255){
                Vec6f patch = _regionCalculator->regionOutput.at<Vec6f>(i, j);
                // cout << patch << endl;
                Vector3 up = {0,0,1};
                Vector3 dir = {0.01f*patch[0], 0.01f*patch[1], 0.01f*patch[2]};
                Vector3 axis = Math::cross(up, dir).normalized();
                Rad angle = Math::acos(Math::dot(up, dir)/(up.length()*dir.length()));

                auto &plane = _sensor->addChild<Object3D>();
                plane.scale({0.0004,0.0004,0.0004});
                // printf("(%.2lf,%.2lf,%.2lf)\n", patch[3], patch[4], patch[5]);
                plane.translate({0.01f* patch[3], 0.01f * patch[4], 0.01f* patch[5]});
                plane.transformLocal(Matrix4::rotationX(-Rad{180.0_degf}));
                if (!isnan(axis.x()) && !isnan(axis.y()) && !isnan(axis.z())){
                    Magnum::Quaternion quat = Magnum::Quaternion::rotation(angle, axis);
                    plane.transformLocal(Matrix4(quat.toMatrix()));
                }
                new RedCubeDrawable{plane, &_drawables, Primitives::planeSolid(), {0.2, 0.0f, 0.3f}};
            }

        }
    }


}

void MyApplication::tickEvent() {
    // auto &cube = _scene.addChild<Object3D>();
    // cube.translate({0.01, 0.01, 0.01});
    // printf("%f,%f,%f\n",0.01,0.01,0.01);
    // cube.scale({0.01, 0.01, 0.01});
    // new RedCubeDrawable{cube, &_drawables};
}

void MyApplication::mousePressEvent(MouseEvent &event) {
    if (event.button() == MouseEvent::Button::Right) {
        // std::cout << "RightMouseButton" << std::endl;
    }
    event.setAccepted();
}

void MyApplication::mouseMoveEvent(MouseMoveEvent &event) {
    if ((event.buttons() & MouseMoveEvent::Button::Left)) {
        Vector2 delta = 4.0f * Vector2{event.relativePosition()} / Vector2{windowSize()};
        _camGrandParent->transformLocal(Matrix4::rotationY(-Rad{delta.x()}));
        _camOriginCube->transformLocal(_scene.absoluteTransformation());
        _camParent->transformLocal(Matrix4::rotationX(-Rad{delta.y()}));
    }
    if ((event.buttons() & MouseMoveEvent::Button::Right)) {
        Vector2 delta = 4.0f * Vector2{event.relativePosition()} / Vector2{windowSize()};
        _camGrandParent->translateLocal({-0.1f*delta.x(), 0, -0.1f*delta.y()});
        _camOriginCube->translateLocal({-0.1f*delta.x(), 0, -0.1f*delta.y()});
    }
    if ((event.buttons() & MouseMoveEvent::Button::Middle)) {
        Vector2 delta = 4.0f * Vector2{event.relativePosition()} / Vector2{windowSize()};
        _camGrandParent->translateLocal({0, 0.1f*delta.y(), 0});
        _camOriginCube->translateLocal({0, 0.1f*delta.y(), 0});
    }
    event.setAccepted();
}

void MyApplication::mouseScrollEvent(MouseScrollEvent &event) {
    Vector2 delta = Vector2{event.offset()};
    _camObject->translate({0, 0, -0.01f * delta.y()});
}

void MyApplication::drawEvent() {
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth);
    _camera->draw(_drawables);

    swapBuffers();
    redraw();
}

int main(int argc, char **argv) {
    MyApplication app({argc, argv});
    return app.exec();

    // PlanarRegionCalculator regionCalculator;
    // regionCalculator.init_opencl();
    // regionCalculator.launch_tester();

}

