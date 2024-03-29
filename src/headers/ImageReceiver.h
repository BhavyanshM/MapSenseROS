//
// Created by quantum on 5/14/21.
//

#ifndef IMAGERECEIVER_H
#define IMAGERECEIVER_H

#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "cv_bridge/cv_bridge.h"

#include "Core.h"
#include "ROS1TopicReceiver.h"
//#include "ImageTools.h"


using namespace sensor_msgs;

class ImageReceiver : public ROS1TopicReceiver
{
   private:
      std::string _imageEncoding;
      ImageConstPtr _imageMessage;
      CameraInfoConstPtr _cameraInfoMessage;
      CompressedImageConstPtr _compressedImageMessage;
      ros::Subscriber *_imageSubscriber;
      ros::Subscriber *_cameraInfoSubscriber;
      float _imageBrightness = 40;
      float _imageOffset = 100;

   public:

      bool _compressed = false;
      bool _cameraInfoSet = false;
      bool _infoMessageSaved = false;
      cv::Mat _image;

   public:
      ImageReceiver(ros::NodeHandle *nh, std::string imageTopic, std::string cameraInfoTopic, bool compressed = false);

      void processMessage(ApplicationState& app) override;

      ImageReceiver();

   public:

      void getData(cv::Mat& image, ApplicationState& app, double& timestamp);

      void render(ApplicationState& app) override;

      void ImGuiUpdate() override;

      void imageCallback(const sensor_msgs::ImageConstPtr& colorMsg);

      void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& compressedMsg);

      void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& message);
};

#endif //IMAGERECEIVER_H
