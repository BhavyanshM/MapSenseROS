#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/Shaders/Phong.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/MeshTools/Interleave.h>
#include <iostream>
#include <Magnum/GL/Renderer.h>

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
    Object3D *_cameraObject;
    Object3D *_cameraVPObject;
    SceneGraph::Camera3D *_camera;
    SceneGraph::DrawableGroup3D _drawables;
    int count = 0;
};

class RedCubeDrawable : public SceneGraph::Drawable3D {
public:
    explicit RedCubeDrawable(Object3D &object, SceneGraph::DrawableGroup3D *group) :
            SceneGraph::Drawable3D{object, group} {
        _mesh = MeshTools::compile(Primitives::cubeSolid());
    }

private:
    GL::Mesh _mesh;
    Shaders::Phong _shader;

    void draw(const Matrix4 &transformationMatrix, SceneGraph::Camera3D &camera) override {
        using namespace Math::Literals;

        _shader.setDiffuseColor(0xa5c9ea_rgbf)
                .setLightColor(Color3{1.0f})
                .setLightPosition({7.0f, 5.0f, 2.5f})
                .setAmbientColor({0.2, 0.0f, 0.3f})
                .setTransformationMatrix(transformationMatrix)
                .setNormalMatrix(transformationMatrix.normalMatrix())
                .setProjectionMatrix(camera.projectionMatrix())
                .draw(_mesh);
    }
};

MyApplication::MyApplication(const Arguments &arguments) : Platform::Application{arguments} {
    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);

    using namespace Math::Literals;

    /* Configure camera */
    _cameraVPObject = new Object3D{&_scene};
    _cameraObject = new Object3D{_cameraVPObject};
    _cameraObject->translate(Vector3::zAxis(0.5f));
    _camera = new SceneGraph::Camera3D(*_cameraObject);
    _camera->setProjectionMatrix(
            Matrix4::perspectiveProjection(35.0_degf,
                                           1.33f, 0.001f, 100.0f));

    /* TODO: Prepare your objects here and add them to the scene */
    auto &cube = _scene.addChild<Object3D>();
    cube.scale({0.01, 0.01, 0.0001});
    new RedCubeDrawable{cube, &_drawables};;

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
        _cameraVPObject->transform(Matrix4::rotationY(-Rad{delta.x()}));
        _cameraVPObject->transformLocal(Matrix4::rotationX(-Rad{delta.y()}));

        // std::cout << "MouseEvent" << std::endl;
    }

    event.setAccepted();
}

void MyApplication::mouseScrollEvent(MouseScrollEvent &event) {
    Vector2 delta = Vector2{event.offset()};
    _cameraObject->translate({0, 0, -0.1f * delta.y()});
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
}

