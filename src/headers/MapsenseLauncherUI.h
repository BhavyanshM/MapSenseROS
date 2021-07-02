//
// Created by quantum on 12/24/20.
//

#ifndef MAPSENSE_LAUNCHER_UI_H
#define MAPSENSE_LAUNCHER_UI_H

#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/ImGuiIntegration/Context.h>
#include <Magnum/ImGuiIntegration/Context.hpp>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/Axis.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/Shaders/PhongGL.h>

#include <Magnum/GL/Mesh.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Math/Quaternion.h>
#include <Magnum/GL/Renderer.h>

#include "MapsenseHeaders.h"
#include "MeshGenerator.h"
#include "SLAMModule.h"
#include "PlanarRegionCalculator.h"
#include "ApplicationState.h"
#include "ImGuiLayout.h"
#include "AppUtils.h"
#include "KeypointDetector.h"

using namespace Magnum::Math::Literals;

   class MyApplication : public Magnum::Platform::Application
{
   public:
      explicit MyApplication(const Arguments& arguments);

      void init(int argc, char** argv);

      ApplicationState appState;

      AppUtils appUtils;



   private:
      void drawEvent() override;

      void tickEvent() override;

      void exitEvent(ExitEvent& event) override;

      void mousePressEvent(MouseEvent& event) override;

      void mouseReleaseEvent(MouseEvent& event) override;

      void mouseMoveEvent(MouseMoveEvent& event) override;

      void mouseScrollEvent(MouseScrollEvent& event) override;

      void viewportEvent(ViewportEvent& event) override;

      void generate_patches();

      void draw_patches();

      void draw_regions();

      Magnum::ImGuiIntegration::Context _imgui{Magnum::NoCreate};
      Magnum::Color4 _clearColor = 0x72909aff_rgbaf;
      float _floatValue = 0.0f;

      Scene3D _scene;
      Object3D *_camGrandParent;
      Object3D *_camParent;
      Object3D *_camObject;
      Object3D *_camOriginCube;
      Object3D *_world;
      Object3D *_sensorAxes;
      Magnum::SceneGraph::Camera3D *_camera;

      vector<Object3D *> planePatches;
      vector<Object3D *> regionEdges;

      Magnum::SceneGraph::DrawableGroup3D _drawables;

      MeshGenerator* _mesher;
      PlanarRegionCalculator *_regionCalculator;
      KeypointDetector* _keypointDetector;
      SLAMModule *_slamModule;
      NetworkManager *_networkManager;
      int count = 0;
      int frameId = 0;

};

void MyApplication::drawEvent()
{
   Magnum::GL::defaultFramebuffer.clear(Magnum::GL::FramebufferClear::Color | Magnum::GL::FramebufferClear::Depth);
   _camera->draw(_drawables);

   _imgui.newFrame();

   ImGui::Text("MapSense");


      /* Dockable Window **************************************************************************************** */
      /* Use for docking. */

      static ImGuiID dockspaceID = 0;
      bool active = true;
      if (ImGui::Begin("Master Window", &active))
      {
         ImGui::TextUnformatted("DockSpace below");
      }
      if (active)
      {
         // Declare Central dockspace
         dockspaceID = ImGui::GetID("HUB_DockSpace");
         ImGui::DockSpace(dockspaceID, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_None | ImGuiDockNodeFlags_PassthruCentralNode /*|ImGuiDockNodeFlags_NoResize*/ );
      }

      bool demo = true;
      ImGui::ShowDemoWindow(&demo);

      ImGui::End();

      ImGui::SetNextWindowDockID(dockspaceID, ImGuiCond_FirstUseEver);
      if (ImGui::Begin("Dockable Window"))
      {
         ImGui::TextUnformatted("Test");
      }
      ImGui::End();

      /* Dockable Window **************************************************************************************** */

   if (ImGui::BeginTabBar("Configuration"))
   {
      if (ImGui::BeginTabItem("Application"))
      {
         ImGuiLayout::getImGuiAppLayout(appState);
         ImGui::EndTabItem();
      }
      ImGui::EndTabBar();
   }
   if (ImGui::BeginTabBar("Modules"))
   {
      if (ImGui::BeginTabItem("Planar Regions"))
      {
         /* Display 2D */
         _regionCalculator->ImGuiUpdate(appState);
         ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("SLAM"))
      {
         _slamModule->ImGuiUpdate();
         ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Network"))
      {
         _networkManager->ImGuiUpdate();
         ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Extras"))
      {
         /* Display 3D */
         ImGui::SameLine(180);
         if (ImGui::Button("Clear Patches"))
            MeshGenerator::clearMesh(planePatches);

         /* Beta Features */
         if (ImGui::Button("Save All"))
         {
            AppUtils::capture_data(ros::package::getPath("map_sense"), "/Extras/Images/Capture", _regionCalculator->inputDepth, _regionCalculator->inputColor,
                                   _regionCalculator->filteredDepth, _regionCalculator->_mapFrameProcessor.debug, appState,
                                   _regionCalculator->planarRegionList);
         }
         if (ImGui::Button("Save Regions"))
         {
            GeomTools::saveRegions(_regionCalculator->planarRegionList, ros::package::getPath("map_sense") + "/Extras/Regions/" +
                                                                        string(4 - to_string(frameId).length(), '0').append(to_string(frameId)) + ".txt");
            frameId++;
         }

         if (ImGui::Button("Configure Memory"))
            AppUtils::checkMemoryLimits();

         ImGui::Checkbox("Stereo-Left", &appState.SHOW_STEREO_LEFT);
         ImGui::Checkbox("Stereo-Right", &appState.SHOW_STEREO_RIGHT);

         if (ImGui::Button("Log OpenCV Build Info"))
         {
            cout << getBuildInformation() << endl;
         }

         ImGui::EndTabItem();
      }
      ImGui::EndTabBar();
   }


   /* Update application cursor */
   _imgui.updateApplicationCursor(*this);

   Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::Blending);
   Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::ScissorTest);
   Magnum::GL::Renderer::disable(Magnum::GL::Renderer::Feature::DepthTest);
   Magnum::GL::Renderer::disable(Magnum::GL::Renderer::Feature::FaceCulling);

   _imgui.drawFrame();

   Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::DepthTest);
   // Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::FaceCulling);
   Magnum::GL::Renderer::disable(Magnum::GL::Renderer::Feature::ScissorTest);
   Magnum::GL::Renderer::disable(Magnum::GL::Renderer::Feature::Blending);

   swapBuffers();
   redraw();
}

void MyApplication::viewportEvent(ViewportEvent& event)
{
   Magnum::GL::defaultFramebuffer.setViewport({{}, event.framebufferSize()});
   _imgui.relayout(Magnum::Vector2{event.windowSize()} / event.dpiScaling(), event.windowSize(), event.framebufferSize());
}

void MyApplication::mousePressEvent(MouseEvent& event)
{
   if (_imgui.handleMousePressEvent(event))
   {
      return;
   }
}

void MyApplication::mouseReleaseEvent(MouseEvent& event)
{
   if (_imgui.handleMouseReleaseEvent(event))
      return;
}

void MyApplication::mouseMoveEvent(MouseMoveEvent& event)
{
   if (_imgui.handleMouseMoveEvent(event))
   {
      return;
   }
   if ((event.buttons() & MouseMoveEvent::Button::Left))
   {
      Magnum::Vector2 delta = 4.0f * Magnum::Vector2{event.relativePosition()} / Magnum::Vector2{windowSize()};
      _camGrandParent->transformLocal(Magnum::Matrix4::rotationY(-Magnum::Rad{delta.x()}));
      _camOriginCube->transformLocal(_scene.absoluteTransformation());
      _camParent->transformLocal(Magnum::Matrix4::rotationX(-Magnum::Rad{delta.y()}));
   }
   if ((event.buttons() & MouseMoveEvent::Button::Right))
   {
      Magnum::Vector2 delta = 4.0f * Magnum::Vector2{event.relativePosition()} / Magnum::Vector2{windowSize()};
      _camGrandParent->translateLocal({-0.5f * delta.x(), 0, -0.5f * delta.y()});
      _camOriginCube->translateLocal({-0.5f * delta.x(), 0, -0.5f * delta.y()});
   }
   if ((event.buttons() & MouseMoveEvent::Button::Middle))
   {
      Magnum::Vector2 delta = 4.0f * Magnum::Vector2{event.relativePosition()} / Magnum::Vector2{windowSize()};
      _camGrandParent->translateLocal({0, 0.8f * delta.y(), 0});
      _camOriginCube->translateLocal({0, 0.8f * delta.y(), 0});
   }
   event.setAccepted();
}

void MyApplication::mouseScrollEvent(MouseScrollEvent& event)
{
   if (_imgui.handleMouseScrollEvent(event))
   {
      /* Prevent scrolling the page */
      return;
   }
   Magnum::Vector2 delta = Magnum::Vector2{event.offset()};
   _camObject->translate({0, 0, -0.5f * delta.y()});
   event.setAccepted();
}

void MyApplication::exitEvent(ExitEvent& event)
{
   _networkManager->camLeft->release();
   _networkManager->camRight->release();
   event.setAccepted(true);
}

int main(int argc, char **argv)
{
   MyApplication app({argc, argv});
   std::vector<std::string> args(argv, argv + argc);

   for (int i = 0; i < argc; i++)
   {
      if (args[i] == "--export")
      {
         printf("Setting EXPORT_REGIONS: true\n");
         app.appState.EXPORT_REGIONS = true;
      }
      if (args[i] == "--kernel-level")
      {
         printf("Setting KERNEL_LEVEL_SLIDER: %d\n", stoi(args[i + 1]));
         app.appState.KERNEL_SLIDER_LEVEL = stoi(args[i + 1]);
      }
      if (args[i] == "--stereo-driver")
      {
         printf("Setting STEREO_DRIVER: true\n");
         app.appState.STEREO_DRIVER = true;
      }
      if (args[i] == "--depth-aligned")
      {
         printf("Setting DEPTH_ALIGNED: true\n");
         app.appState.DEPTH_ALIGNED = true;
      }
      if (args[i] == "--camera-name")
      {
         printf("Setting TOPIC_CAMERA_NAME: %s\n", args[i + 1].c_str());
         app.appState.TOPIC_CAMERA_NAME = args[i + 1];
      }
   }

   app.init(argc, argv);
   return app.exec();
}



#endif //MAPSENSE_LAUNCHER_UI_H
