//
// Created by quantum on 5/14/21.
//

#ifndef IMAGERECEIVER_H
#define IMAGERECEIVER_H

#include "ROS1TopicReceiver.h"

class ImageReceiver : public ROS1TopicReceiver
{
   private:
      String imageEncoding;
      ImageConstPtr imageMessage;
      CameraInfoConstPtr cameraInfoMessage;
      CompressedImageConstPtr compressedImageMessage;
      Subscriber imageSubscriber, cameraInfoSubscriber;
      bool compressed = false;
      bool cameraInfoSet = false;

   public:
      Mat image;

   public:
      ImageReceiver(String imageTopic, String imageEncoding, String cameraInfoTopic, bool compressed = false);

      void processMessage(ApplicationState& app) override;

      ImageReceiver();

   public:
      void render() override;

      void imageCallback(const sensor_msgs::ImageConstPtr& colorMsg);

      void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& compressedMsg);

      void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& message);
};

#endif //IMAGERECEIVER_H
