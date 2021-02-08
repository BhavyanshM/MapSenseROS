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
    _dataReceiver = new NetworkManager();
    _dataReceiver->init_ros_node(arguments.argc, arguments.argv);
    _regionCalculator = new PlanarRegionCalculator(appState);
    _regionCalculator->initOpenCL(appState);

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
    _sensorAxes->scale({0.1,0.1,0.1});
    new RedCubeDrawable{*_sensorAxes, &_drawables, Primitives::axis3D(), {0.5, 0.1f, 0.1f}};

    _camObject->translate({0,0,10.0f});
    _sensor->transformLocal(Matrix4::rotationX(Rad{90.0_degf}));

    _camOriginCube = new Object3D{_camGrandParent};
    _camOriginCube->scale({0.01f, 0.01f, 0.01f});
    new RedCubeDrawable{*_camOriginCube, &_drawables, Primitives::cubeSolid(), {0.2, 0.0f, 0.3f}};

    namedWindow("DebugOutput", WINDOW_NORMAL);

    // setMinimalLoopPeriod(32); /* Needs to be less than 30-32 milliseconds for real-time performance */

    /* --------------------------------------- For Testing Purposes Only --------------------------------------------*/
//    generate_patches();

}

void clear(vector<Object3D*>& objects){
    for(int i = 0; i<objects.size(); i++){
        delete objects[i];
    }
    objects.clear();
}

void displayDebugOutput(Mat disp){

    resizeWindow("DebugOutput", (int)(disp.cols), (int)(disp.rows));
    imshow("DebugOutput", disp);
    waitKey(1);
}

void MyApplication::tickEvent() {
//     cout << "TickEvent:" << count++ << endl;
    switch(_displayItem){
        case SHOW_INPUT_COLOR : displayDebugOutput(_regionCalculator->inputColor); break;
        case SHOW_INPUT_DEPTH : {
            Mat dispDepth;
            _regionCalculator->getInputDepth(dispDepth, _showGraph);
            displayDebugOutput(dispDepth); break;
        }
        case SHOW_REGION_COMPONENTS : displayDebugOutput(_regionCalculator->mapFrameProcessor.debug); break;
        case SHOW_FILTERED_DEPTH : {
            Mat dispDepth;
            _regionCalculator->getFilteredDepth(dispDepth, _showGraph);
            displayDebugOutput(dispDepth);
        } break;
        case -1 : destroyAllWindows(); break;
    }
    if(_rosEnabled){
        _dataReceiver->spin_ros_node();
        _regionCalculator->generateRegions(_dataReceiver, appState);
        if(_showRegionEdges){
            clear(regionEdges);
            draw_regions();
        }
    }
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
        _camGrandParent->translateLocal({-0.5f*delta.x(), 0, -0.5f*delta.y()});
        _camOriginCube->translateLocal({-0.5f*delta.x(), 0, -0.5f*delta.y()});
    }
    if ((event.buttons() & MouseMoveEvent::Button::Middle)) {
        Vector2 delta = 4.0f * Vector2{event.relativePosition()} / Vector2{windowSize()};
        _camGrandParent->translateLocal({0, 0.8f*delta.y(), 0});
        _camOriginCube->translateLocal({0, 0.8f*delta.y(), 0});
    }
    event.setAccepted();
}

void MyApplication::mouseScrollEvent(MouseScrollEvent &event) {
    if(_imgui.handleMouseScrollEvent(event)) {
        /* Prevent scrolling the page */
        return;
    }
    Vector2 delta = Vector2{event.offset()};
    _camObject->translate({0, 0, -0.5f * delta.y()});
    event.setAccepted();
}



void MyApplication::draw_patches(){
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
                plane.scale({appState.MAGNUM_PATCH_SCALE, appState.MAGNUM_PATCH_SCALE, appState.MAGNUM_PATCH_SCALE });
                plane.translate({patch[3],patch[4],patch[5]});
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

void MyApplication::draw_regions(){
    auto start = high_resolution_clock::now();
    for(int i = 0; i<_regionCalculator->currentRegionList.size(); i++){

         shared_ptr<PlanarRegion> planarRegion = _regionCalculator->currentRegionList[i];
//         Vector3f normal = planarRegion->getPCANormal();
//         Vector3f center = planarRegion->getMeanCenter();
        // printf("Region[%d]:(%d), Center:(%.3lf, %.3lf, %.3lf), Normal:(%.3lf, %.3lf, %.3lf), Vertices:(%d)\n", planarRegion->getId(), planarRegion->getNumPatches(), center[0], center[1], center[2], axis[0], axis[1], axis[2], planarRegion->getNumOfBoundaryVertices());
        vector<Vector3f> vertices = planarRegion->getVertices();
        for(int j = appState.NUM_SKIP_EDGES; j<vertices.size(); j+=appState.NUM_SKIP_EDGES){
            Object3D& edge = _sensor->addChild<Object3D>();
            Vector3f prevPoint = vertices[j-appState.NUM_SKIP_EDGES];
            Vector3f curPoint = vertices[j];
            regionEdges.emplace_back(&edge);
            new RedCubeDrawable{edge, &_drawables, Primitives::line3D({prevPoint.x(), prevPoint.y(), prevPoint.z()},{curPoint.x(), curPoint.y(), curPoint.z()}),
                                {(planarRegion->getId() * 123 % 255) / 255.0f, (planarRegion->getId() * 161 % 255) / 255.0f, (planarRegion->getId() * 113 % 255) / 255.0f}};
        }
    }
    auto end = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(end - start).count();
    ROS_INFO("Visualization Took: %.2f ms", duration / (float) 1000);

}

void MyApplication::generate_patches(){
    _regionCalculator->launch_tester(appState);
    draw_patches();
}


void MyApplication::drawEvent() {
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth);
    _camera->draw(_drawables);

    _imgui.newFrame();

    ImGui::Text("MapSense");

    if (ImGui::ColorEdit3("Color", _clearColor.data())) { GL::Renderer::setClearColor(_clearColor); }

    ImGui::SliderInt("Kernel Level", &appState.KERNEL_SLIDER_LEVEL, 2, 10); appState.update();
    ImGui::Text("Input:%d,%d Patch:%d,%d Level:%d", appState.INPUT_HEIGHT,  appState.INPUT_WIDTH,
                appState.PATCH_HEIGHT, appState.PATCH_WIDTH, appState.KERNEL_SLIDER_LEVEL);

    if(ImGui::Button("Region Components")) _displayItem = 0;
    ImGui::SameLine(140);
    ImGui::Checkbox("Boundary", &appState.SHOW_BOUNDARIES);
    ImGui::SameLine(220);
    ImGui::Checkbox("Internal", &appState.SHOW_PATCHES);
    if(ImGui::Button("Input Color")) _displayItem = 3;
    if(ImGui::Button("Input Depth")) _displayItem = 1;
    if(ImGui::Button("Filtered Depth")) _displayItem = 2;
    ImGui::SameLine(180);
    ImGui::Checkbox("Graph", &_showGraph);

    if(ImGui::Button("Hide Display")) { destroyAllWindows();destroyAllWindows();_displayItem = -1; }

    ImGui::SliderInt("Region Boundary Diff", &appState.REGION_BOUNDARY_DIFF, 10, 40);
    ImGui::SliderInt("Region Min Patches", &appState.REGION_MIN_PATCHES, 4, 100);
    ImGui::SliderFloat("Magnum Patch Scale", &appState.MAGNUM_PATCH_SCALE, 0.001f, 0.04f);
    ImGui::SliderFloat("Filter Disparity Threshold", &appState.FILTER_DISPARITY_THRESHOLD, 1000, 4000);
    ImGui::SliderFloat("Merge Distance Threshold", &appState.MERGE_DISTANCE_THRESHOLD, 0.01, 0.16);
    ImGui::SliderFloat("Merge Angular Threshold", &appState.MERGE_ANGULAR_THRESHOLD, 0.2f, 1.0f);

    if(ImGui::BeginTabBar("Tab Bar")){
        if(ImGui::BeginTabItem("Launcher")) {
            if (ImGui::Button("Generate Patches")) {
                clear(planePatches);
                generate_patches();
            }
            ImGui::SameLine(180);
            if (ImGui::Button("Clear Patches")) { clear(planePatches); }
            ImGui::EndTabItem();
        }
        if(ImGui::BeginTabItem("ROS")){
            if(ImGui::Button("Enable ROS Node")){_rosEnabled = true;}
            ImGui::SameLine(180);
            if(ImGui::Button("Disable ROS Node")){_rosEnabled = false;}
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::Checkbox("Show Edges", &_showRegionEdges);
    ImGui::SliderInt("Skip Edges", &appState.NUM_SKIP_EDGES, 1, 20);

    ImGui::Text("Time:%.3f ms FPS:%.1f", 1000.0/Double(ImGui::GetIO().Framerate), Double(ImGui::GetIO().Framerate));

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
    // regionCalculator.initOpenCL();
    // regionCalculator.launch_tester();

}

