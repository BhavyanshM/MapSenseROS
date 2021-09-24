//
// Created by quantum on 9/23/21.
//

#ifndef MAP_SENSE_POINTCLOUDRECEIVER_H
#define MAP_SENSE_POINTCLOUDRECEIVER_H

#include "ROS1TopicReceiver.h"
#include "sensor_msgs/PointCloud2.h"
#include "torch/torch.h"

class PointCloudReceiver : public ROS1TopicReceiver
{
   private:
      sensor_msgs::PointCloud2ConstPtr _cloudMessage;
      Subscriber *_cloudSubscriber;
      torch::Tensor _cloud;

   public:
      PointCloudReceiver() = default;

      PointCloudReceiver(NodeHandle *nh, std::string cloudTopic, bool compressed = false);

      void processMessage(ApplicationState& app) override;

      void getData(cv::Mat& image, ApplicationState& app, double& timestamp);

      void render() override;

      void ImGuiUpdate() override;

      void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg);

};

#endif //MAP_SENSE_POINTCLOUDRECEIVER_H
