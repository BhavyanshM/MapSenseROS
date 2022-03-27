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
#include "Scene/Mesh/PointCloud.h"
#include "PointCloudReceiver.h"


#include "Core.h"
#include <ApplicationState.h>


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
      ros::NodeHandle *rosNode;
      cv::VideoCapture *camLeft;
      cv::VideoCapture *camRight;

      std::unordered_map<std::string, ROS1TopicReceiver*> receivers;
      ros::Subscriber subMapSenseParams;
      ros::Subscriber subSLAMPose;
      ros::Publisher planarRegionPub;
      ros::Publisher slamPosePub;
      ros::Publisher rawPlanesPub;
      ros::Publisher coloredCloudPub;

      bool depthCamInfoSet = false;
      bool paramsAvailable = false;
      bool nextDepthAvailable = false;
      bool nextColorAvailable = false;

      NetworkManager(ApplicationState app, AppUtils* appUtils);

      std::vector<TopicInfo> getROSTopicList();

      void publishSamplePose(int count);

      void PublishPlanes(const std::vector<std::shared_ptr<PlanarRegion>>& regions, int poseId);

      void GetTopicSelection(std::vector<TopicInfo> topics, TopicInfo& currentTopic);

      void ImGuiUpdate(ApplicationState& appState);

      int AddReceiver(TopicInfo data, TopicInfo info = TopicInfo());

      void ReceiverUpdate(ApplicationState& app);

      void load_next_frame(cv::Mat& depth, cv::Mat& color, double& timestamp, ApplicationState& app);

      void depthCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& message);

      void colorCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& message);

      void depthCallback(const ImageConstPtr& depthMsg);

      void colorCallback(const sensor_msgs::ImageConstPtr& colorMsg, std::string name);

      void colorCompressedCallback(const sensor_msgs::CompressedImageConstPtr& colorMsg);

      void MapsenseParamsCallback(const map_sense::MapsenseConfiguration compressedMsg);

      void SLAMPoseCallback(const geometry_msgs::PoseStamped poseMsg);

      void InitNode(int argc, char **argv, ApplicationState& app);

      void SpinNode();

      void load_next_stereo_frame(cv::Mat& left, cv::Mat& right, ApplicationState& app);

      void PublishPoseStamped(RigidBodyTransform worldToSensorTransform, int id);

      void AcceptMapsenseConfiguration(ApplicationState& appState);

//      void PublishColoredPointCloud(Clay::Ref<Clay::PointCloud> cloud);
};

#endif //SRC_SENSORDATARECEIVER_H
