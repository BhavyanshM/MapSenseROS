//
// Created by quantum on 9/23/21.
//

#include "PointCloudReceiver.h"

PointCloudReceiver::PointCloudReceiver(NodeHandle *nh, std::string cloudTopic, bool compressed)
{
   CLAY_LOG_INFO("PointCloudReceiver Created!");
   topicName = cloudTopic;
   _cloudSubscriber = new Subscriber();
   _cloudToRender = std::make_shared<Clay::PointCloud>(glm::vec4(0.8f, 0.3f, 0.3f, 1.0f), nullptr);

   //   *(this->_cloudSubscriber) = nh->subscribe(cloudTopic, 2, &PointCloudReceiver::cloudCallback, this);
   *(this->_cloudSubscriber) = nh->subscribe(cloudTopic, 2, &PointCloudReceiver::cloudCallback, this);
}

void PointCloudReceiver::processMessage(ApplicationState& app)
{
   ROS_DEBUG("PointCloud Processing!");
}

void PointCloudReceiver::getData(cv::Mat& image, ApplicationState& app, double& timestamp)
{

}

void PointCloudReceiver::render()
{
   if (_available && _renderEnabled)
   {
      uint32_t numPoints = 0;
//      CLAY_LOG_INFO("Added all new vertices. 1 {}", _cloudToRender->GetSize());
      _cloudToRender->Reset();
//      CLAY_LOG_INFO("Added all new vertices. 2 {}", _cloudToRender->GetSize());
      for(const pcl::PointXYZ& pt : _cloudMessage->points)
      {
         _cloudToRender->Insert(-pt.y, pt.z, -pt.x);
         numPoints++;
      }
      _available = false;
//      CLAY_LOG_INFO("Added all new vertices. 3 {}", _cloudToRender->GetSize());
   }
   if(_renderEnabled) _cloudToRender->Upload();
}

void PointCloudReceiver::ImGuiUpdate()
{
}

void PointCloudReceiver::cloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloudMsg)
{
   CLAY_LOG_INFO("PointCloudCallback.");
   if (!_available)
   {
      _cloudMessage = cloudMsg;
      _available = true;
      if(_saveScans) {
         FileManager::WriteScanPoints(_cloudMessage, _scanCount);
         _available = false;
         _scanCount++;
      }
      CLAY_LOG_INFO("New PointCloud Processing! FrameId:{}, Width:{}, Height:{}", _cloudMessage->header.frame_id.c_str(), _cloudMessage->width, _cloudMessage->height);
   }
}
