//
// Created by quantum on 9/23/21.
//

#ifndef MAP_SENSE_POINTCLOUDRECEIVER_H
#define MAP_SENSE_POINTCLOUDRECEIVER_H

#include "ROS1TopicReceiver.h"
#include "sensor_msgs/PointCloud2.h"

using namespace sensor_msgs;

class PointCloudReceiver : public ROS1TopicReceiver
{
   private:
      PointCloud2ConstPtr _cloudMessage;
      Subscriber *_cloudSubscriber;

   public:
      PointCloudReceiver() = default;

      PointCloudReceiver(NodeHandle *nh, String cloudTopic, bool compressed = false);

      void processMessage(ApplicationState& app) override;

      void getData(Mat& image, ApplicationState& app, double& timestamp);

      void render() override;

      void ImGuiUpdate() override;

      void cloudCallback(const PointCloud2ConstPtr& cloudMsg);

};

#endif //MAP_SENSE_POINTCLOUDRECEIVER_H
