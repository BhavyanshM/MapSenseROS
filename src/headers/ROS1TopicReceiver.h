//
// Created by quantum on 5/14/21.
//

#ifndef SENSORSTREAMRECEIVER_H
#define SENSORSTREAMRECEIVER_H

#include <ApplicationState.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

#include "Core/Core.h"
#include "AppUtils.h"
#include "imgui.h"
#include "MapsenseHeaders.h"

using namespace ros;
using namespace std;
using namespace cv;


class ROS1TopicReceiver
{
   protected:
      AppUtils* appUtils;
      String topicName;
      bool messageReceived = false;
      bool renderingEnabled = false;
      bool undistortEnabled = false;
      double timestampLastReceived = 0.0;

   public:
      virtual void render() = 0;
      virtual void ImGuiUpdate() = 0;
      virtual void processMessage(ApplicationState& app){};
      void setAppUtils(AppUtils* appUtils);
};

#endif //SENSORSTREAMRECEIVER_H
