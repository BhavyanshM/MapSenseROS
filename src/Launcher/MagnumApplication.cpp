//
// Created by quantum on 2/20/21.
//

#include "MagnumApplication.h"

typedef Magnum::Matrix4 Mat4;
typedef Magnum::Vector2 Vec2;

MagnumApplication::MagnumApplication(const Arguments& arguments) : Magnum::Platform::Application{arguments,
                                                                                         Configuration{}.setTitle("Magnum ImGui Application").setSize(
                                                                                               Magnum::Vector2i(1024, 768)).setWindowFlags(
                                                                                               Configuration::WindowFlag::Resizable)}
{

   /* Set up proper blending to be used by ImGui. There's a great chance
      you'll need this exact behavior for the rest of your scene. If not, set
      this only for the drawFrame() call. */
   Magnum::GL::Renderer::setBlendEquation(Magnum::GL::Renderer::BlendEquation::Add, Magnum::GL::Renderer::BlendEquation::Add);
   Magnum::GL::Renderer::setBlendFunction(Magnum::GL::Renderer::BlendFunction::SourceAlpha, Magnum::GL::Renderer::BlendFunction::OneMinusSourceAlpha);

   /* TODO: Instantiate the Information Processors */

   /* TODO: Check that the appropriate flags for renderer are set*/
   Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::DepthTest);
   // Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::FaceCulling);

   /* TODO: Configure Camera in Scene Graph*/
   _camGrandParent = new Object3D{&_scene};
   _sensor = new Object3D{&_scene};
   _camParent = new Object3D{_camGrandParent};
   _camObject = new Object3D{_camParent};
   _camera = new Magnum::SceneGraph::Camera3D(*_camObject);
   _camera->setProjectionMatrix(Mat4::perspectiveProjection(35.0_degf, 1.33f, 0.001f, 100.0f));
   _sensor->transformLocal(Mat4::rotationX(Magnum::Rad{90.0_degf}));

   /* TODO: Prepare your objects here and add them to the scene */
   _sensorAxes = new Object3D{_sensor};
   _sensorAxes->scale({0.1, 0.1, 0.1});
   new RedCubeDrawable{*_sensorAxes, &_drawables, Magnum::Primitives::axis3D(), {0.5, 0.1f, 0.1f}};

   _camObject->translate({0, 0, 10.0f});
   _sensor->transformLocal(Mat4::rotationX(Magnum::Rad{90.0_degf}));

   _camOriginCube = new Object3D{_camGrandParent};
   _camOriginCube->scale({0.01f, 0.01f, 0.01f});
   new RedCubeDrawable{*_camOriginCube, &_drawables, Magnum::Primitives::cubeSolid(), {0.2, 0.0f, 0.3f}};
}

void MagnumApplication::viewportEvent(ViewportEvent& event)
{
   Magnum::GL::defaultFramebuffer.setViewport({{}, event.framebufferSize()});
}

void MagnumApplication::mouseMoveEvent(MouseMoveEvent& event)
{
   if ((event.buttons() & MouseMoveEvent::Button::Left))
   {
      Vec2 delta = 4.0f * Vec2{event.relativePosition()} / Vec2{windowSize()};
      _camGrandParent->transformLocal(Mat4::rotationY(-Magnum::Rad{delta.x()}));
      _camOriginCube->transformLocal(_scene.absoluteTransformation());
      _camParent->transformLocal(Mat4::rotationX(-Magnum::Rad{delta.y()}));
   }
   if ((event.buttons() & MouseMoveEvent::Button::Right))
   {
      Vec2 delta = 4.0f * Vec2{event.relativePosition()} / Vec2{windowSize()};
      _camGrandParent->translateLocal({-0.5f * delta.x(), 0, -0.5f * delta.y()});
      _camOriginCube->translateLocal({-0.5f * delta.x(), 0, -0.5f * delta.y()});
   }
   if ((event.buttons() & MouseMoveEvent::Button::Middle))
   {
      Vec2 delta = 4.0f * Vec2{event.relativePosition()} / Vec2{windowSize()};
      _camGrandParent->translateLocal({0, 0.8f * delta.y(), 0});
      _camOriginCube->translateLocal({0, 0.8f * delta.y(), 0});
   }
   event.setAccepted();
}

void MagnumApplication::mouseScrollEvent(MouseScrollEvent& event)
{
   Vec2 delta = Vec2{event.offset()};
   _camObject->translate({0, 0, -0.5f * delta.y()});
   event.setAccepted();
}

void MagnumApplication::drawEvent()
{
   Magnum::GL::defaultFramebuffer.clear(Magnum::GL::FramebufferClear::Color | Magnum::GL::FramebufferClear::Depth);
   _camera->draw(_drawables);

   draw();

   Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::Blending);
   Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::ScissorTest);
   Magnum::GL::Renderer::disable(Magnum::GL::Renderer::Feature::DepthTest);
   Magnum::GL::Renderer::disable(Magnum::GL::Renderer::Feature::FaceCulling);

   Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::DepthTest);
   // Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::FaceCulling);
   Magnum::GL::Renderer::disable(Magnum::GL::Renderer::Feature::ScissorTest);
   Magnum::GL::Renderer::disable(Magnum::GL::Renderer::Feature::Blending);

   swapBuffers();
   redraw();
}

