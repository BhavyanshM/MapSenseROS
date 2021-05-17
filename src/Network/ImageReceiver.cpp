//
// Created by quantum on 5/14/21.
//

#include "ImageReceiver.h"

ImageReceiver::ImageReceiver() : ROS1TopicReceiver()
{
}

ImageReceiver::ImageReceiver(NodeHandle *nh, String imageTopic, String cameraInfoTopic, bool compressed) : ROS1TopicReceiver()
{
   this->topicName = imageTopic;
   this->compressed = compressed;
   this->imageEncoding = imageEncoding;

   this->imageSubscriber = new Subscriber();
   this->cameraInfoSubscriber = new Subscriber();

   if (!compressed)
   {
      *(this->imageSubscriber) = nh->subscribe(imageTopic, 2, &ImageReceiver::imageCallback, this);
      *(this->cameraInfoSubscriber) = nh->subscribe(cameraInfoTopic, 2, &ImageReceiver::cameraInfoCallback, this);
   } else
   {
      *(this->imageSubscriber) = nh->subscribe(imageTopic, 2, &ImageReceiver::compressedImageCallback, this);
   }
}

void ImageReceiver::compressedImageCallback(const CompressedImageConstPtr& compressedMsg)
{
   ROS_DEBUG("Compressed Image Callback: ", compressedMsg->header.stamp);
   compressedImageMessage = compressedMsg;
}

void ImageReceiver::imageCallback(const ImageConstPtr& colorMsg)
{
   ROS_DEBUG("Image Callback; ", colorMsg->header.stamp);
   imageMessage = colorMsg;
}

void ImageReceiver::cameraInfoCallback(const CameraInfoConstPtr& message)
{
   ROS_DEBUG("Camera Info Callback: ", message->header.stamp);
   cameraInfoMessage = message;
}

void ImageReceiver::render()
{
   ROS_DEBUG("Render: %s", this->topicName.c_str());
   if (renderingEnabled)
   {
      if(imageEncoding == sensor_msgs::image_encodings::TYPE_16UC1)
      {
         this->image.convertTo(this->image, -1, this->imageBrightness, this->imageOffset);
         cvtColor(this->image, this->image, COLOR_GRAY2BGR);
      }
      appUtils->appendToDebugOutput(this->image);
   }
}

void ImageReceiver::ImGuiUpdate()
{
   ImGui::Text("%s", (String("ROS1 Receiver: ") + topicName).c_str());
   ImGui::Checkbox((String("Render: ") + topicName).c_str(), &renderingEnabled);
}

void ImageReceiver::processMessage(ApplicationState& app)
{
   ROS_DEBUG("Process Message");
   cv_bridge::CvImagePtr img_ptr;
   if (imageMessage != nullptr || compressedImageMessage != nullptr)
   {
      ROS_DEBUG("Image Message Received");
      try
      {
         if (compressed)
         {
            ROS_INFO("Compressed Image Being Decoded.", compressedImageMessage->header.stamp);
            image = imdecode(cv::Mat(compressedImageMessage->data), 1);
            timestampLastReceived = compressedImageMessage.get()->header.stamp.toSec();
         } else
         {
            if(imageMessage->encoding == "mono8")
               this->imageEncoding = sensor_msgs::image_encodings::MONO8;
            if(imageMessage->encoding == "16UC1")
               this->imageEncoding = sensor_msgs::image_encodings::TYPE_16UC1;
            if(imageMessage->encoding.find("rgb8") != string::npos)
               this->imageEncoding = sensor_msgs::image_encodings::TYPE_16UC3;
            img_ptr = cv_bridge::toCvCopy(*imageMessage, imageEncoding);
            image = img_ptr->image;
            timestampLastReceived = imageMessage.get()->header.stamp.toSec();
         }
         ROS_DEBUG("Image Processed:", image.rows, image.cols);
      } catch (cv_bridge::Exception& e)
      {
         ROS_ERROR("Could not convert to image! %s", e.what());
      }
   }
   if (cameraInfoMessage != nullptr)
   {
      ROS_DEBUG("DEPTH_SET:", cameraInfoSet);
      cameraInfoSet = true;
      app.INPUT_WIDTH = cameraInfoMessage->width / app.DIVISION_FACTOR;
      app.INPUT_HEIGHT = cameraInfoMessage->height / app.DIVISION_FACTOR;
      app.DEPTH_FX = cameraInfoMessage->K[0] / app.DIVISION_FACTOR;
      app.DEPTH_FY = cameraInfoMessage->K[4] / app.DIVISION_FACTOR;
      app.DEPTH_CX = cameraInfoMessage->K[2] / app.DIVISION_FACTOR;
      app.DEPTH_CY = cameraInfoMessage->K[5] / app.DIVISION_FACTOR;
      app.update();
   }
}
