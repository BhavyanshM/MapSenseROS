//
// Created by quantum on 9/17/21.
//

#ifndef MAP_SENSE_MAPSENSELAYER_H
#define MAP_SENSE_MAPSENSELAYER_H

#include "Core/Application.h"
#include "Core/Clay.h"
#include "FileManager.h"
#include "Core.h"
#include "MapsenseHeaders.h"
#include "SLAMModule.h"
#include "PlanarRegionCalculator.h"
#include "ApplicationState.h"
#include "ImGuiLayout.h"
#include "AppUtils.h"
#include "KeypointDetector.h"
#include "OpenCLManager.h"
#include "IterativeClosestPoint.h"

namespace Clay
{
   class MapsenseLayer : public Layer
   {
      public:
         MapsenseLayer(int argc, char** argv);

         ~MapsenseLayer() = default;

         void OnAttach() override;

         void OnDetach() override;

         void OnUpdate(Timestep ts) override;

         void MapsenseUpdate();

         void OnEvent(Event& e) override;

         void OnImGuiRender() override;

         ApplicationState appState;

      private:
         Ref<FrameBuffer> _frameBuffer;
         CameraController _cameraController;
         glm::vec4 _squareColor;
         Ref<Texture2D> _texture;
         Ref<Texture2D> _checkerTexture;
         std::vector<Ref<Model>> _models;
         Ref<Model> _rootPCL;

         struct ProfileResult
         {
            const char *Name;
            float Time;
         };

         std::vector<ProfileResult> _profileResults;
         glm::vec2 _viewportSize = {0,0};

         bool _viewportFocused = false;
         bool _viewportHovered = false;
         bool dockspaceOpen = true;
         bool opt_fullscreen;
         uint32_t count = 0;
         uint32_t frameId = 0;
         uint32_t _scanCount = 0;

         ImGuiDockNodeFlags dockspace_flags;
         ImGuiWindowFlags window_flags;

         AppUtils appUtils;
         IterativeClosestPoint* _icp;
         PlanarRegionCalculator *_regionCalculator;
         KeypointDetector* _keypointDetector;
         SLAMModule *_slamModule;
         NetworkManager *_networkManager;
         OpenCLManager *_openCLManager;
         PointCloudReceiver* _pclReceiver;
   };
}

#endif //MAP_SENSE_MAPSENSELAYER_H
