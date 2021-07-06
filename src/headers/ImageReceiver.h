//
// Created by quantum on 5/14/21.
//

#ifndef IMAGERECEIVER_H
#define IMAGERECEIVER_H

#include "Core.h"
#include "ROS1TopicReceiver.h"

class ImageReceiver : public ROS1TopicReceiver
{
   private:
      String imageEncoding;
      ImageConstPtr imageMessage;
      CameraInfoConstPtr cameraInfoMessage;
      CompressedImageConstPtr compressedImageMessage;
      Subscriber *imageSubscriber;
      Subscriber *cameraInfoSubscriber;
      float imageBrightness = 40;
      float imageOffset = 100;

   public:

      bool compressed = false;
      bool cameraInfoSet = false;
      Mat image;

   public:
      ImageReceiver(NodeHandle *nh, String imageTopic, String cameraInfoTopic, bool compressed = false);

      void processMessage(ApplicationState& app) override;

      ImageReceiver();

   public:

      void getData(Mat& image, ApplicationState& app, double& timestamp);

      void render() override;

      void ImGuiUpdate() override;

      void imageCallback(const sensor_msgs::ImageConstPtr& colorMsg);

      void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& compressedMsg);

      void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& message);
};

#endif //IMAGERECEIVER_H
