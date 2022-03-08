//
// Created by quantum on 5/14/21.
//

#include "ImageReceiver.h"
#include "ImageTools.h"
#include "Core/Log.h"

ImageReceiver::ImageReceiver() : ROS1TopicReceiver()
{
}

ImageReceiver::ImageReceiver(NodeHandle *nh, std::string imageTopic, std::string cameraInfoTopic, bool compressed) : ROS1TopicReceiver()
{
   this->topicName = imageTopic;
   this->_compressed = compressed;

   this->_imageSubscriber = new Subscriber();
   this->_cameraInfoSubscriber = new Subscriber();

   if (!compressed)
   {
      *(this->_imageSubscriber) = nh->subscribe(imageTopic, 2, &ImageReceiver::imageCallback, this);
      *(this->_cameraInfoSubscriber) = nh->subscribe(cameraInfoTopic, 4, &ImageReceiver::cameraInfoCallback, this);
   } else
   {
      *(this->_imageSubscriber) = nh->subscribe(imageTopic, 2, &ImageReceiver::compressedImageCallback, this);
   }
}

void ImageReceiver::compressedImageCallback(const CompressedImageConstPtr& compressedMsg)
{
       ROS_DEBUG("Compressed Image Callback: %s", topicName.c_str());
       _compressedImageMessage = compressedMsg;
}

void ImageReceiver::imageCallback(const ImageConstPtr& colorMsg)
{
       ROS_DEBUG("Image Callback: %s", topicName.c_str());
       _imageMessage = colorMsg;
}

void ImageReceiver::cameraInfoCallback(const CameraInfoConstPtr& message)
{
   ROS_DEBUG("CameraParams Info Callback: %s", message->distortion_model.c_str());
   if(!_infoMessageSaved)
   {
      _cameraInfoMessage = message;
      _infoMessageSaved = true;
      CLAY_LOG_INFO("Depth Info Message Saved: {}", _infoMessageSaved);
   }
}

void ImageReceiver::render(ApplicationState& app)
{
   MAPSENSE_PROFILE_FUNCTION();
   ROS_DEBUG("Render: %s", this->topicName.c_str());
   if (renderingEnabled)
   {
      if(_imageEncoding == sensor_msgs::image_encodings::TYPE_16UC1)
      {
         this->_image.convertTo(this->_image, -1, this->_imageBrightness, this->_imageOffset);
         cvtColor(this->_image, this->_image, cv::COLOR_GRAY2BGR);
      }
      cv::Mat disp;
      if(undistortEnabled){

         disp = ImageTools::cvUndistort(this->_image, cv::Mat(), cv::Mat());
      } else
         disp = this->_image;
      AppUtils::DisplayImage(disp, app);
   }
}

void ImageReceiver::ImGuiUpdate()
{
   ImGui::Text("%s", (std::string("ROS1 Receiver: ") + topicName).c_str());
   ImGui::Checkbox((std::string("Render: ") + topicName).c_str(), &renderingEnabled);
   ImGui::Checkbox((std::string("Undistort: ") + topicName).c_str(), &undistortEnabled);
}

void ImageReceiver::processMessage(ApplicationState& app)
{
   MAPSENSE_PROFILE_FUNCTION();
   ROS_DEBUG("Process Message");
   cv_bridge::CvImagePtr img_ptr;
   if (_imageMessage != nullptr || _compressedImageMessage != nullptr)
   {
      ROS_DEBUG("Image Message Received");
      try
      {
         if (_compressed)
         {
            ROS_DEBUG("Compressed Image Being Decoded.", _compressedImageMessage->header.stamp);
            _image = imdecode(cv::Mat(_compressedImageMessage->data), 1);
            timestampLastReceived = _compressedImageMessage.get()->header.stamp.toSec();
         } else
         {
            if(_imageMessage->encoding == "mono8")
               this->_imageEncoding = sensor_msgs::image_encodings::MONO8;
            else if(_imageMessage->encoding == "16UC1")
               this->_imageEncoding = sensor_msgs::image_encodings::TYPE_16UC1;
            else if(_imageMessage->encoding.find("rgb8") != string::npos)
               this->_imageEncoding = sensor_msgs::image_encodings::TYPE_16UC3;
            img_ptr = cv_bridge::toCvCopy(*_imageMessage, _imageEncoding);

            if(undistortEnabled)
               _image = ImageTools::cvUndistort(img_ptr->image, cv::Mat(), cv::Mat());
            else
               _image = img_ptr->image;

            timestampLastReceived = _imageMessage.get()->header.stamp.toSec();
         }
         ROS_DEBUG("Image Processed:", _image.rows, _image.cols);
      } catch (cv_bridge::Exception& e)
      {
         ROS_ERROR("Could not convert to _image! %s", e.what());
      }
   }
}

void ImageReceiver::getData(cv::Mat& image, ApplicationState& app, double& timestamp)
{
   MAPSENSE_PROFILE_FUNCTION();
    ROS_DEBUG("Image Received: %s", topicName.c_str());
   timestamp = this->timestampLastReceived;
   if(this->_image.rows != 0 && this->_image.cols != 0 && !this->_image.empty())
   {
      image = this->_image.clone();
   }
   //   CLAY_LOG_INFO("Receiver: {}, DEPTH_SET: {}, Message Saved: {}", topicName, _cameraInfoSet, _infoMessageSaved);
   if (_cameraInfoMessage != nullptr && _infoMessageSaved)
   {
      //      CLAY_LOG_INFO("Copying Depth Info Message: {}", _cameraInfoMessage->distortion_model);
      _cameraInfoSet = true;
      app.DEPTH_INPUT_WIDTH = _cameraInfoMessage->width / app.DIVISION_FACTOR;
      app.DEPTH_INPUT_HEIGHT = _cameraInfoMessage->height / app.DIVISION_FACTOR;
      app.DEPTH_FX = _cameraInfoMessage->K[0] / app.DIVISION_FACTOR;
      app.DEPTH_FY = _cameraInfoMessage->K[4] / app.DIVISION_FACTOR;
      app.DEPTH_CX = _cameraInfoMessage->K[2] / app.DIVISION_FACTOR;
      app.DEPTH_CY = _cameraInfoMessage->K[5] / app.DIVISION_FACTOR;
      app.update();
   }
}
