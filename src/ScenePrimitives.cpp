#include "ScenePrimitives.h"

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

    /* TODO: Instantiate the Information Processors */
    _dataReceiver = new SensorDataReceiver();
    // _dataReceiver->init_ros_node(arguments.argc, arguments.argv);

    _regionCalculator = new PlanarRegionCalculator();
    _regionCalculator->init_opencl();




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
    _camOriginCube->scale({0.0001f, 0.0001f, 0.0001f});
    new RedCubeDrawable{*_camOriginCube, &_drawables, Primitives::cubeSolid(), {0.2, 0.0f, 0.3f}};

    // setMinimalLoopPeriod(32); /* Needs to be less than 30-32 milliseconds for real-time performance */

    /* --------------------------------------- For Testing Purposes Only --------------------------------------------*/
    generate_patches();

}

void MyApplication::tickEvent() {
    // cout << "TickEvent:" << count++ << endl;

    // _dataReceiver->spin_ros_node();
    // _regionCalculator->generate_regions(_dataReceiver);
    //

    //
    // for(int i = 0; i<_regionCalculator->planarRegionList.size(); i++){
    //     shared_ptr<PlanarRegion> planarRegion = _regionCalculator->planarRegionList[i];
    //     Vector3f normal = _regionCalculator->planarRegionList[i]->getNormal();
    //     Vector3f center = _regionCalculator->planarRegionList[i]->getCenter();
    //     Vector3 up = {0,0,-1};
    //     Vector3 dir = {normal[0], normal[1], -normal[2]};
    //     Vector3 axis = Math::cross(up, dir).normalized();
    //     Rad angle = Math::acos(Math::dot(up, dir)/(up.length()*dir.length()));
    //
    //
    //     auto &region = _sensor->addChild<Object3D>();
    //     // float regionScale = 0.001 * (float) planarRegion->getNumPatches();
    //     float regionScale = 1.2f;
    //     region.scale({ regionScale*0.004f, regionScale*0.004f, regionScale*0.004f});
    //     region.translate({ 0.01f*center[0], 0.01f*center[1], 0.01f*center[2]});
    //     // region.transformLocal(Matrix4::rotationX(-Rad{180.0_degf}));
    //
    //     axis = axis.normalized();
    //     // printf("Region[%d]:(%d), Center:(%.3lf, %.3lf, %.3lf), Normal:(%.3lf, %.3lf, %.3lf), Vertices:(%d)\n", planarRegion->getId(), planarRegion->getNumPatches(), center[0], center[1], center[2], axis[0], axis[1], axis[2], planarRegion->getNumOfVertices());
    //     if (!isnan(axis.x()) && !isnan(axis.y()) && !isnan(axis.z()) && axis.isNormalized()){
    //         Magnum::Quaternion quat = Magnum::Quaternion::rotation(angle, axis);
    //         region.transformLocal(Matrix4(quat.toMatrix()));
    //     }
    //     // GL::Buffer bufferToPack;
    //     // MeshGenerator::getPlanarRegionBuffer(planarRegion, bufferToPack);
    //     cout << "REACHED:" << _regionCalculator->planarRegionList.size() << endl;
    //     new RedCubeDrawable{region, &_drawables, MeshGenerator::getPlanarRegionMesh(planarRegion) , {1.0f, 0.0f, 0.3f}};
    // }
    //
    // for(int i = 0; i < _regionCalculator->output.getRegionOutput().rows; i++){
    //     for(int j = 0; j < _regionCalculator->output.getRegionOutput().cols; j++){
    //         uint8_t edges = _regionCalculator->output.getPatchData().at<uint8_t>(i, j);
    //         if (edges == 255){
    //             Vec6f patch = _regionCalculator->output.getRegionOutput().at<Vec6f>(i, j);
    //             // cout << patch << endl;
    //             Vector3 up = {0,0,1};
    //             Vector3 dir = {0.01f*patch[0], 0.01f*patch[1], 0.01f*patch[2]};
    //             Vector3 axis = Math::cross(up, dir).normalized();
    //             Rad angle = Math::acos(Math::dot(up, dir)/(up.length()*dir.length()));
    //
    //             auto &plane = _sensor->addChild<Object3D>();
    //             plane.scale({0.0004,0.0004,0.0004});
    //             plane.translate({0.01f* patch[3], 0.01f * patch[4], 0.01f* patch[5]});
    //             // plane.transformLocal(Matrix4::rotationX(-Rad{180.0_degf}));
    //             if (!isnan(axis.x()) && !isnan(axis.y()) && !isnan(axis.z())){
    //                 Magnum::Quaternion quat = Magnum::Quaternion::rotation(angle, axis);
    //                 plane.transformLocal(Matrix4(quat.toMatrix()));
    //             }
    //             new RedCubeDrawable{plane, &_drawables, Primitives::planeSolid(), {0.4, 0.4f, 0.6f}};
    //         }
    //     }
    // }

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

void clear(vector<Object3D*>& objects){
    for(int i = 0; i<objects.size(); i++){
        delete objects[i];
    }
    objects.clear();
}

void MyApplication::generate_patches(){
    _regionCalculator->launch_tester(appState.getDepthFile(), appState.getColorFile());
//     for(int i = 0; i<_regionCalculator->planarRegionList.size(); i++){
//         shared_ptr<PlanarRegion> planarRegion = _regionCalculator->planarRegionList[i];
//         Vector3f normal = _regionCalculator->planarRegionList[i]->getNormal();
//         Vector3f center = _regionCalculator->planarRegionList[i]->getCenter();
//         Vector3 up = {0,0,-1};
//         Vector3 dir = {normal[0], normal[1], -normal[2]};
//         Vector3 axis = Math::cross(up, dir).normalized();
//         Rad angle = Math::acos(Math::dot(up, dir)/(up.length()*dir.length()));
//
//
//         auto &region = _sensor->addChild<Object3D>();
//         // float regionScale = 0.001 * (float) planarRegion->getNumPatches();
//         float regionScale = 1.2f;
//         region.scale({ regionScale*0.004f, regionScale*0.004f, regionScale*0.004f});
//         region.translate({ 0.01f*center[0], 0.01f*center[1], 0.01f*center[2]});
//         // region.transformLocal(Matrix4::rotationX(-Rad{180.0_degf}));
//
//         axis = axis.normalized();
//         // printf("Region[%d]:(%d), Center:(%.3lf, %.3lf, %.3lf), Normal:(%.3lf, %.3lf, %.3lf), Vertices:(%d)\n", planarRegion->getId(), planarRegion->getNumPatches(), center[0], center[1], center[2], axis[0], axis[1], axis[2], planarRegion->getNumOfVertices());
//         if (!isnan(axis.x()) && !isnan(axis.y()) && !isnan(axis.z()) && axis.isNormalized()){
//             Magnum::Quaternion quat = Magnum::Quaternion::rotation(angle, axis);
//             region.transformLocal(Matrix4(quat.toMatrix()));
//         }
//         // GL::Buffer bufferToPack;
//         // MeshGenerator::getPlanarRegionBuffer(planarRegion, bufferToPack);
//         cout << "REACHED:" << _regionCalculator->planarRegionList.size() << endl;
//         new RedCubeDrawable{region, &_drawables, MeshGenerator::getPlanarRegionMesh(planarRegion) , {(planarRegion->getId() * 123 % 255)/255.0f, (planarRegion->getId() * 161 % 255)/255.0f, (planarRegion->getId() * 113 % 255)/255.0f}};
//     }

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

                Object3D& plane = _sensor->addChild<Object3D>();
                planePatches.emplace_back(&plane);
                plane.scale({0.0002,0.0002,0.0002});
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

void MyApplication::drawEvent() {
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth);
    _camera->draw(_drawables);

    _imgui.newFrame();

    ImGui::Text("MapSense");
    ImGui::SliderFloat("Float", &_floatValue, 0.0f, 1.0f);
    if(ImGui::ColorEdit3("Color", _clearColor.data()))
        GL::Renderer::setClearColor(_clearColor);
    if(ImGui::Button("Clear Patches"))
        clear(planePatches);
    if(ImGui::Button("Generate Patches"))
        generate_patches();
    ImGui::Text("Time:%.3f ms FPS:%.1f",
                1000.0/Double(ImGui::GetIO().Framerate), Double(ImGui::GetIO().Framerate));

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

