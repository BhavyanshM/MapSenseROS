//
// Created by quantum on 5/14/21.
//

#include "ImageReceiver.h"

ImageReceiver::ImageReceiver() : ROS1TopicReceiver()
{

}

ImageReceiver::ImageReceiver(NodeHandle* nh, String imageTopic, String imageEncoding, String cameraInfoTopic, bool compressed) : ROS1TopicReceiver()
{
   this->compressed = compressed;
   this->imageEncoding = imageEncoding;

   this->imageSubscriber = new Subscriber();
   this->cameraInfoSubscriber = new Subscriber();

   if(!compressed)
   {
      *(this->imageSubscriber) = nh->subscribe(imageTopic, 2, &ImageReceiver::imageCallback, this);
      *(this->cameraInfoSubscriber) = nh->subscribe(cameraInfoTopic, 2, &ImageReceiver::cameraInfoCallback, this);
   }else
   {
      *(this->imageSubscriber) = nh->subscribe(imageTopic, 2, &ImageReceiver::compressedImageCallback, this);
   }
}

void ImageReceiver::compressedImageCallback(const CompressedImageConstPtr& compressedMsg)
{
   ROS_INFO("Compressed Image Callback: ", compressedMsg->header.stamp);
   compressedImageMessage = compressedMsg;
}

void ImageReceiver::imageCallback(const ImageConstPtr& colorMsg)
{
   ROS_INFO("Image Callback; ", colorMsg->header.stamp);
   imageMessage = colorMsg;
}

void ImageReceiver::cameraInfoCallback(const CameraInfoConstPtr& message)
{
   ROS_INFO("Camera Info Callback: ", message->header.stamp);
   cameraInfoMessage = message;
}

void ImageReceiver::render()
{
   ROS_INFO("Render");
   appUtils->appendToDebugOutput(this->image);
}

void ImageReceiver::processMessage(ApplicationState& app)
{
   ROS_INFO("Process Message");
   cv_bridge::CvImagePtr img_ptr;
   if (imageMessage != nullptr || compressedImageMessage != nullptr)
   {
      ROS_INFO("Image Message Received");
      try
      {
         if (compressed)
         {
            image = imdecode(cv::Mat(compressedImageMessage->data), 1);
            timestampLastReceived = compressedImageMessage.get()->header.stamp.toSec();
         } else
         {
            img_ptr = cv_bridge::toCvCopy(*imageMessage, imageEncoding);
            image = img_ptr->image;
            timestampLastReceived = imageMessage.get()->header.stamp.toSec();
         }
         ROS_INFO("Image Processed:", image.rows, image.cols);
      } catch (cv_bridge::Exception& e)
      {
         ROS_ERROR("Could not convert to image! %s", e.what());
      }
   }
   if (cameraInfoMessage != nullptr)
   {
      ROS_INFO("DEPTH_SET:", cameraInfoSet);
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
