//
// Created by quantum on 2/20/21.
//

#include "MagnumApplication.h"

MagnumApplication::MagnumApplication(const Arguments& arguments) : Platform::Application{arguments,
                                                                                         Configuration{}.setTitle("Magnum ImGui Application").setSize(
                                                                                               Magnum::Vector2i(1024, 768)).setWindowFlags(
                                                                                               Configuration::WindowFlag::Resizable)}
{

   /* Set up proper blending to be used by ImGui. There's a great chance
      you'll need this exact behavior for the rest of your scene. If not, set
      this only for the drawFrame() call. */
   GL::Renderer::setBlendEquation(GL::Renderer::BlendEquation::Add, GL::Renderer::BlendEquation::Add);
   GL::Renderer::setBlendFunction(GL::Renderer::BlendFunction::SourceAlpha, GL::Renderer::BlendFunction::OneMinusSourceAlpha);

   /* TODO: Instantiate the Information Processors */

   /* TODO: Check that the appropriate flags for renderer are set*/
   GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
   // GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);

   /* TODO: Configure Camera in Scene Graph*/
   _camGrandParent = new Object3D{&_scene};
   _sensor = new Object3D{&_scene};
   _camParent = new Object3D{_camGrandParent};
   _camObject = new Object3D{_camParent};
   _camera = new SceneGraph::Camera3D(*_camObject);
   _camera->setProjectionMatrix(Matrix4::perspectiveProjection(35.0_degf, 1.33f, 0.001f, 100.0f));
   _sensor->transformLocal(Matrix4::rotationX(Rad{90.0_degf}));

   /* TODO: Prepare your objects here and add them to the scene */
   _sensorAxes = new Object3D{_sensor};
   _sensorAxes->scale({0.1, 0.1, 0.1});
   new RedCubeDrawable{*_sensorAxes, &_drawables, Primitives::axis3D(), {0.5, 0.1f, 0.1f}};

   _camObject->translate({0, 0, 10.0f});
   _sensor->transformLocal(Matrix4::rotationX(Rad{90.0_degf}));

   _camOriginCube = new Object3D{_camGrandParent};
   _camOriginCube->scale({0.01f, 0.01f, 0.01f});
   new RedCubeDrawable{*_camOriginCube, &_drawables, Primitives::cubeSolid(), {0.2, 0.0f, 0.3f}};
}

void MagnumApplication::viewportEvent(ViewportEvent& event)
{
   GL::defaultFramebuffer.setViewport({{}, event.framebufferSize()});
}

void MagnumApplication::mouseMoveEvent(MouseMoveEvent& event)
{
   if ((event.buttons() & MouseMoveEvent::Button::Left))
   {
      Vector2 delta = 4.0f * Vector2{event.relativePosition()} / Vector2{windowSize()};
      _camGrandParent->transformLocal(Matrix4::rotationY(-Rad{delta.x()}));
      _camOriginCube->transformLocal(_scene.absoluteTransformation());
      _camParent->transformLocal(Matrix4::rotationX(-Rad{delta.y()}));
   }
   if ((event.buttons() & MouseMoveEvent::Button::Right))
   {
      Vector2 delta = 4.0f * Vector2{event.relativePosition()} / Vector2{windowSize()};
      _camGrandParent->translateLocal({-0.5f * delta.x(), 0, -0.5f * delta.y()});
      _camOriginCube->translateLocal({-0.5f * delta.x(), 0, -0.5f * delta.y()});
   }
   if ((event.buttons() & MouseMoveEvent::Button::Middle))
   {
      Vector2 delta = 4.0f * Vector2{event.relativePosition()} / Vector2{windowSize()};
      _camGrandParent->translateLocal({0, 0.8f * delta.y(), 0});
      _camOriginCube->translateLocal({0, 0.8f * delta.y(), 0});
   }
   event.setAccepted();
}

void MagnumApplication::mouseScrollEvent(MouseScrollEvent& event)
{
   Vector2 delta = Vector2{event.offset()};
   _camObject->translate({0, 0, -0.5f * delta.y()});
   event.setAccepted();
}

void MagnumApplication::drawEvent()
{
   GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth);
   _camera->draw(_drawables);

   draw();

   GL::Renderer::enable(GL::Renderer::Feature::Blending);
   GL::Renderer::enable(GL::Renderer::Feature::ScissorTest);
   GL::Renderer::disable(GL::Renderer::Feature::DepthTest);
   GL::Renderer::disable(GL::Renderer::Feature::FaceCulling);

   GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
   // GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
   GL::Renderer::disable(GL::Renderer::Feature::ScissorTest);
   GL::Renderer::disable(GL::Renderer::Feature::Blending);

   swapBuffers();
   redraw();
}

