//
// Created by quantum on 9/23/21.
//

#include "PointCloudReceiver.h"

PointCloudReceiver::PointCloudReceiver(NodeHandle *nh, String cloudTopic, bool compressed)
{
   this->topicName = cloudTopic;
   this->_cloudSubscriber = new Subscriber();

   *(this->_cloudSubscriber) = nh->subscribe(cloudTopic, 2, &PointCloudReceiver::cloudCallback, this);
}

void PointCloudReceiver::processMessage(ApplicationState& app)
{
   ROS1TopicReceiver::processMessage(app);
}

void PointCloudReceiver::getData(Mat& image, ApplicationState& app, double& timestamp)
{
}

void PointCloudReceiver::render()
{
}

void PointCloudReceiver::ImGuiUpdate()
{
}

void PointCloudReceiver::cloudCallback(const PointCloud2ConstPtr& cloudMsg)
{
   _cloudMessage = cloudMsg;
   CLAY_LOG_INFO("PointCloud Received! FrameId:{}, Width:{}, Height:{}", _cloudMessage->header.frame_id, _cloudMessage->width, _cloudMessage->height);
}
