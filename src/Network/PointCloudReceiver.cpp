//
// Created by quantum on 9/23/21.
//

#include "PointCloudReceiver.h"

PointCloudReceiver::PointCloudReceiver(NodeHandle *nh, std::string cloudTopic, bool compressed)
{
   this->topicName = cloudTopic;
   this->_cloudSubscriber = new Subscriber();

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
}

void PointCloudReceiver::ImGuiUpdate()
{
}

void PointCloudReceiver::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
   _cloudMessage = cloudMsg;
   ROS_DEBUG("PointCloud Processing! FrameId:{}, Width:{}, Height:{}", _cloudMessage->header.frame_id, _cloudMessage->width, _cloudMessage->height);
}
