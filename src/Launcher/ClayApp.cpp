//
// Created by quantum on 9/17/21.
//

#include "ClayApp.h"

ClayApp::ClayApp(int argc, char** argv)
{
   //   PushLayer(new ExampleLayer());
   Clay::MapsenseLayer* app = new Clay::MapsenseLayer(argc, argv);
   std::vector<std::string> args(argv, argv + argc);
   for (int i = 0; i < argc; i++)
   {
      if (args[i] == "--export")
      {
         printf("Setting EXPORT_REGIONS: true\n");
         app->appState.EXPORT_REGIONS = true;
      }
      if (args[i] == "--kernel-level")
      {
         printf("Setting KERNEL_LEVEL_SLIDER: %d\n", stoi(args[i + 1]));
         app->appState.KERNEL_SLIDER_LEVEL = stoi(args[i + 1]);
      }
      if (args[i] == "--stereo-driver")
      {
         printf("Setting STEREO_DRIVER: true\n");
         app->appState.STEREO_DRIVER = true;
      }
      if (args[i] == "--depth-aligned")
      {
         printf("Setting DEPTH_ALIGNED: true\n");
         app->appState.DEPTH_ALIGNED = true;
      }
      if (args[i] == "--camera-name")
      {
         printf("Setting TOPIC_CAMERA_NAME: %s\n", args[i + 1].c_str());
         app->appState.TOPIC_CAMERA_NAME = args[i + 1];
      }
   }
   PushLayer(app);
}

int main(int argc, char** argv)
{
   Clay::Log::Init();   CLAY_LOG_INFO("Welcome to Clay Engine!");

   CLAY_PROFILE_BEGIN_SESSION("Startup", "ClayProfile-Startup.json");
   ClayApp app(argc, argv);
   CLAY_PROFILE_END_SESSION();

   CLAY_PROFILE_BEGIN_SESSION("Runtime", "ClayProfile-Runtime.json");
   app.Run();
   CLAY_PROFILE_END_SESSION();

   return 0;
}