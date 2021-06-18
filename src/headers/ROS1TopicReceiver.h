//
// Created by quantum on 5/14/21.
//

#ifndef SENSORSTREAMRECEIVER_H
#define SENSORSTREAMRECEIVER_H

#include <ApplicationState.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "ImageTools.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "cv_bridge/cv_bridge.h"
#include "AppUtils.h"
#include "imgui.h"
#include "MapsenseHeaders.h"

using namespace ros;
using namespace std;
using namespace cv;
using namespace sensor_msgs;

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
      virtual void render(){};
      virtual void ImGuiUpdate(){};
      virtual void processMessage(ApplicationState& app){};
      void setAppUtils(AppUtils* appUtils);
};

#endif //SENSORSTREAMRECEIVER_H
