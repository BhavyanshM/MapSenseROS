//
// Created by quantum on 9/23/21.
//

#include "PointCloudReceiver.h"

PointCloudReceiver::PointCloudReceiver(NodeHandle *nh, std::string cloudTopic, bool compressed)
{
   topicName = cloudTopic;
   _cloudSubscriber = new Subscriber();
   _cloudToRender = std::make_shared<Clay::PointCloud>();
   _cloudToRender->SetColor({0.8f, 0.3f, 0.3f, 1.0f});


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
   if (_available)
   {
      uint32_t numPoints = 0;
      _cloudToRender->Reset();
      for(const pcl::PointXYZ& pt : _cloudMessage->points)
      {
         _cloudToRender->Insert(-pt.y, pt.z, -pt.x);
         numPoints++;
      }
      _available = false;
   }
   _cloudToRender->Upload();
}

void PointCloudReceiver::ImGuiUpdate()
{
}

void PointCloudReceiver::cloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloudMsg)
{
   if (!_available)
   {
      _cloudMessage = cloudMsg;
      _available = true;
   }
   ROS_DEBUG("PointCloud Processing! FrameId:{}, Width:{}, Height:{}", _cloudMessage->header.frame_id.c_str(), _cloudMessage->width, _cloudMessage->height);
}
