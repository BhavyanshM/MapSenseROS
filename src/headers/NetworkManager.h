#ifndef SRC_SENSORDATARECEIVER_H
#define SRC_SENSORDATARECEIVER_H

#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "cv_bridge/cv_bridge.h"

#include "imgui.h"
#include <CL/cl.hpp>
#include "map_sense/RawGPUPlanarRegionList.h"
#include "map_sense/MapsenseConfiguration.h"
#include "ImageReceiver.h"
#include "PointCloudReceiver.h"


#include "MapsenseHeaders.h"
#include "Core.h"
#include <ApplicationState.h>

using namespace ros;
using namespace std;
using namespace chrono;
using namespace cv;
using namespace sensor_msgs;

typedef ros::master::TopicInfo TopicInfo;

class NetworkManager
{
   private:
      TopicInfo currentDataTopic, currentInfoTopic;

   public:
      AppUtils* appUtils;
      CameraInfoConstPtr depthCameraInfo, colorCameraInfo;
      ImageConstPtr colorMessage;
      CompressedImageConstPtr colorCompressedMessage;
      map_sense::MapsenseConfiguration paramsMessage;
      ImageConstPtr depthMessage;
      NodeHandle *rosNode;
      VideoCapture *camLeft;
      VideoCapture *camRight;

      vector<ROS1TopicReceiver*> receivers;
      Subscriber subMapSenseParams;
      Publisher planarRegionPub;
      Publisher slamPosePub;

      bool depthCamInfoSet = false;
      bool paramsAvailable = false;
      bool nextDepthAvailable = false;
      bool nextColorAvailable = false;

      NetworkManager(ApplicationState app, AppUtils* appUtils);

      vector<TopicInfo> getROSTopicList();

      void publishSamplePose(int count);

      void getTopicSelection(vector<TopicInfo> topics, TopicInfo& currentTopic);

      void ImGuiUpdate();

      int addReceiver(TopicInfo data, TopicInfo info = TopicInfo());

      void receiverUpdate(ApplicationState& app);

      void load_next_frame(Mat& depth, Mat& color, double& timestamp, ApplicationState& app);

      void depthCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& message);

      void colorCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& message);

      void depthCallback(const ImageConstPtr& depthMsg);

      void colorCallback(const sensor_msgs::ImageConstPtr& colorMsg, String name);

      void colorCompressedCallback(const sensor_msgs::CompressedImageConstPtr& colorMsg);

      void mapSenseParamsCallback(const map_sense::MapsenseConfiguration compressedMsg);

      void init_ros_node(int argc, char **argv, ApplicationState& app);

      void spin_ros_node();

      void load_next_stereo_frame(Mat& left, Mat& right, ApplicationState& app);

      void publishSLAMPose(RigidBodyTransform pose);

      void acceptMapsenseConfiguration(ApplicationState& appState);
};

#endif //SRC_SENSORDATARECEIVER_H
