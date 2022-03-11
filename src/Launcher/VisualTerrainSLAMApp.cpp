//
// Created by quantum on 9/17/21.
//

#include "VisualTerrainSLAMApp.h"

VisualTerrainSLAMApp::VisualTerrainSLAMApp(int argc, char** argv)
{
   //   PushLayer(new ExampleLayer());
   Clay::NetworkedTerrainSLAMLayer* app = new Clay::NetworkedTerrainSLAMLayer(argc, argv);
   PushLayer(app);
}

int main(int argc, char** argv)
{
   Clay::Log::Init();   CLAY_LOG_INFO("Welcome to Clay Engine!");

   CLAY_PROFILE_BEGIN_SESSION("Startup", "ClayProfile-Startup.json");
   VisualTerrainSLAMApp app(argc, argv);
   CLAY_PROFILE_END_SESSION();

   CLAY_PROFILE_BEGIN_SESSION("Runtime", "ClayProfile-Runtime.json");
   app.Run();
   CLAY_PROFILE_END_SESSION();

   return 0;
}