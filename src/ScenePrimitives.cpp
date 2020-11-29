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
#include <iostream>
#include <Magnum/GL/Renderer.h>
#include <imgui.h>
#include "PlanarRegionCalculator.h"
#include "MeshGenerator.h"

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

    ImGuiIntegration::Context _imgui{NoCreate};
    Color4 _clearColor = 0x72909aff_rgbaf;
    Float _floatValue = 0.0f;
    bool _showDemoWindow = true;
    bool _showAnotherWindow = false;

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

MyApplication::MyApplication(const Arguments &arguments) : Platform::Application{arguments,
                                                                                 Configuration{}.setTitle("Magnum ImGui Application")
                                                                                            .setSize(Magnum::Vector2i(1024, 768))
                                                                                            .setWindowFlags(Configuration::WindowFlag::Resizable)
} {

    _imgui = ImGuiIntegration::Context(Vector2{windowSize()}/dpiScaling(),
                                       windowSize(), framebufferSize());

    /* Set up proper blending to be used by ImGui. There's a great chance
       you'll need this exact behavior for the rest of your scene. If not, set
       this only for the drawFrame() call. */
    GL::Renderer::setBlendEquation(GL::Renderer::BlendEquation::Add,
                                   GL::Renderer::BlendEquation::Add);
    GL::Renderer::setBlendFunction(GL::Renderer::BlendFunction::SourceAlpha,
                                   GL::Renderer::BlendFunction::OneMinusSourceAlpha);

    /* TODO: Instantiate the Information Processor*/
    _regionCalculator = new PlanarRegionCalculator();
    _regionCalculator->init_opencl();
    _regionCalculator->launch_tester();

    /* TODO: Check that the appropriate flags for renderer are set*/
    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    // GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);

    /* TODO: Configure Camera in Scene Graph*/
    _camGrandParent = new Object3D{&_scene};
    _sensor = new Object3D{&_scene};
    _camParent = new Object3D{_camGrandParent};
    _camObject = new Object3D{_camParent};
    _camera = new SceneGraph::Camera3D(*_camObject);
    _camera->setProjectionMatrix(Matrix4::perspectiveProjection(35.0_degf,1.33f, 0.001f, 100.0f));
    _sensor->transformLocal(Matrix4::rotationX(Rad{90.0_degf}));

    /* TODO: Prepare your objects here and add them to the scene */
    _sensorAxes = new Object3D{_sensor};
    _sensorAxes->scale({0.01,0.01,0.01});
    new RedCubeDrawable{*_sensorAxes, &_drawables, Primitives::axis3D(), {0.5, 0.1f, 0.1f}};

    _camObject->translate({0,0,0.4f});
    _sensor->transformLocal(Matrix4::rotationX(Rad{90.0_degf}));

    _camOriginCube = new Object3D{_camGrandParent};
    _camOriginCube->scale({0.0004f, 0.0004f, 0.0004f});
    new RedCubeDrawable{*_camOriginCube, &_drawables, Primitives::cubeSolid(), {0.2, 0.0f, 0.3f}};

    for(int i = 0; i<_regionCalculator->planarRegionList.size(); i++){
        shared_ptr<PlanarRegion> planarRegion = _regionCalculator->planarRegionList[i];
        Vector3f normal = _regionCalculator->planarRegionList[i]->getNormal();
        Vector3f center = _regionCalculator->planarRegionList[i]->getCenter();
        Vector3 up = {0,0,-1};
        Vector3 dir = {normal[0], normal[1], -normal[2]};
        Vector3 axis = Math::cross(up, dir).normalized();
        Rad angle = Math::acos(Math::dot(up, dir)/(up.length()*dir.length()));

        auto &region = _sensor->addChild<Object3D>();
        // float regionScale = 0.001 * (float) planarRegion->getNumPatches();
        float regionScale = 1.2f;
        region.scale({ regionScale*0.004f, regionScale*0.004f, regionScale*0.004f});
        region.translate({ 0.01f*center[0], 0.01f*center[1], 0.01f*center[2]});
        // region.transformLocal(Matrix4::rotationX(-Rad{180.0_degf}));

        axis = axis.normalized();
        printf("Region[%d]:(%d), Center:(%.3lf, %.3lf, %.3lf), Normal:(%.3lf, %.3lf, %.3lf), Vertices:(%d)\n", planarRegion->getId(), planarRegion->getNumPatches(), center[0], center[1], center[2], axis[0], axis[1], axis[2], planarRegion->getNumOfVertices());
        if (!isnan(axis.x()) && !isnan(axis.y()) && !isnan(axis.z()) && axis.isNormalized()){
            Magnum::Quaternion quat = Magnum::Quaternion::rotation(angle, axis);
            region.transformLocal(Matrix4(quat.toMatrix()));
        }
        // GL::Buffer bufferToPack;
        // MeshGenerator::getPlanarRegionBuffer(planarRegion, bufferToPack);
        new RedCubeDrawable{region, &_drawables, MeshGenerator::getPlanarRegionMesh(planarRegion) , {1.0f, 0.0f, 0.3f}};
    }

    for(int i = 0; i < _regionCalculator->output.getRegionOutput().rows; i++){
        for(int j = 0; j < _regionCalculator->output.getRegionOutput().cols; j++){
            uint8_t edges = _regionCalculator->output.getPatchData().at<uint8_t>(i, j);
            if (edges == 255){
                Vec6f patch = _regionCalculator->output.getRegionOutput().at<Vec6f>(i, j);
                // cout << patch << endl;
                Vector3 up = {0,0,1};
                Vector3 dir = {0.01f*patch[0], 0.01f*patch[1], 0.01f*patch[2]};
                Vector3 axis = Math::cross(up, dir).normalized();
                Rad angle = Math::acos(Math::dot(up, dir)/(up.length()*dir.length()));

                auto &plane = _sensor->addChild<Object3D>();
                plane.scale({0.0004,0.0004,0.0004});
                plane.translate({0.01f* patch[3], 0.01f * patch[4], 0.01f* patch[5]});
                // plane.transformLocal(Matrix4::rotationX(-Rad{180.0_degf}));
                if (!isnan(axis.x()) && !isnan(axis.y()) && !isnan(axis.z())){
                    Magnum::Quaternion quat = Magnum::Quaternion::rotation(angle, axis);
                    plane.transformLocal(Matrix4(quat.toMatrix()));
                }
                new RedCubeDrawable{plane, &_drawables, Primitives::planeSolid(), {0.4, 0.4f, 0.6f}};
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

void MyApplication::viewportEvent(ViewportEvent& event) {
    GL::defaultFramebuffer.setViewport({{}, event.framebufferSize()});
    _imgui.relayout(Vector2{event.windowSize()}/event.dpiScaling(),
                    event.windowSize(), event.framebufferSize());
}

void MyApplication::mousePressEvent(MouseEvent &event) {
    if(_imgui.handleMousePressEvent(event)) { return; }
}

void MyApplication::mouseReleaseEvent(MouseEvent& event) {
    if(_imgui.handleMouseReleaseEvent(event)) return;
}

void MyApplication::mouseMoveEvent(MouseMoveEvent &event) {
    if(_imgui.handleMouseMoveEvent(event)) {
        return;
    }
    if ((event.buttons() & MouseMoveEvent::Button::Left)) {
        Vector2 delta = 4.0f * Vector2{event.relativePosition()} / Vector2{windowSize()};
        _camGrandParent->transformLocal(Matrix4::rotationY(-Rad{delta.x()}));
        _camOriginCube->transformLocal(_scene.absoluteTransformation());
        _camParent->transformLocal(Matrix4::rotationX(-Rad{delta.y()}));
    }
    if ((event.buttons() & MouseMoveEvent::Button::Right)) {
        Vector2 delta = 4.0f * Vector2{event.relativePosition()} / Vector2{windowSize()};
        _camGrandParent->translateLocal({-0.05f*delta.x(), 0, -0.05f*delta.y()});
        _camOriginCube->translateLocal({-0.05f*delta.x(), 0, -0.05f*delta.y()});
    }
    if ((event.buttons() & MouseMoveEvent::Button::Middle)) {
        Vector2 delta = 4.0f * Vector2{event.relativePosition()} / Vector2{windowSize()};
        _camGrandParent->translateLocal({0, 0.1f*delta.y(), 0});
        _camOriginCube->translateLocal({0, 0.1f*delta.y(), 0});
    }
    event.setAccepted();
}

void MyApplication::mouseScrollEvent(MouseScrollEvent &event) {
    if(_imgui.handleMouseScrollEvent(event)) {
        /* Prevent scrolling the page */
        return;
    }
    Vector2 delta = Vector2{event.offset()};
    _camObject->translate({0, 0, -0.01f * delta.y()});
    event.setAccepted();
}

void MyApplication::drawEvent() {
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth);
    _camera->draw(_drawables);

    _imgui.newFrame();

    ImGui::Text("Hello, world!");
    ImGui::SliderFloat("Float", &_floatValue, 0.0f, 1.0f);
    if(ImGui::ColorEdit3("Clear Color", _clearColor.data()))
        GL::Renderer::setClearColor(_clearColor);
    if(ImGui::Button("Test Window"))
        _showDemoWindow ^= true;
    if(ImGui::Button("Another Window"))
        _showAnotherWindow ^= true;
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                1000.0/Double(ImGui::GetIO().Framerate), Double(ImGui::GetIO().Framerate));

    // if(_showDemoWindow) {
    //     ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiCond_FirstUseEver);
    //     ImGui::ShowDemoWindow();
    // }

    /* Update application cursor */
    _imgui.updateApplicationCursor(*this);

    GL::Renderer::enable(GL::Renderer::Feature::Blending);
    GL::Renderer::enable(GL::Renderer::Feature::ScissorTest);
    GL::Renderer::disable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::disable(GL::Renderer::Feature::FaceCulling);

    _imgui.drawFrame();

    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    // GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
    GL::Renderer::disable(GL::Renderer::Feature::ScissorTest);
    GL::Renderer::disable(GL::Renderer::Feature::Blending);

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

