//
// Created by quantum on 9/23/21.
//

#ifndef MAP_SENSE_POINTCLOUDRECEIVER_H
#define MAP_SENSE_POINTCLOUDRECEIVER_H

#include "ROS1TopicReceiver.h"
#include "MapsenseHeaders.h"
#include "torch/torch.h"
#include "Core/Core.h"
#include "Scene/Mesh/PointCloud.h"
#include "FileManager.h"

class PointCloudReceiver : public ROS1TopicReceiver
{
   public:

      PointCloudReceiver() = default;
      PointCloudReceiver(NodeHandle *nh, std::string cloudTopic, bool compressed = false);

      void processMessage(ApplicationState& app) override;

      void getData(cv::Mat& image, ApplicationState& app, double& timestamp);

      void render() override;

      void ImGuiUpdate() override;

//      void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg);

      void cloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloudMsg);

      Clay::Ref<Clay::PointCloud> GetRenderable() const {return _cloudToRender;}
      const std::string& GetTopicName() const { return topicName; }
      bool IsReadyToRender() const {return _renderEnabled;}
      void SetReadyToRender(bool ready) { _renderEnabled = ready;}
      void SetRenderEnabled(bool enabled) {_renderEnabled = enabled;}

   private:
//      sensor_msgs::PointCloud2ConstPtr _cloudMessage;
      bool _available = false;
      bool _renderEnabled = false;
      pcl::PointCloud<pcl::PointXYZ>::ConstPtr _cloudMessage;
      Subscriber *_cloudSubscriber;
      torch::Tensor _cloud;
      Clay::Ref<Clay::PointCloud> _cloudToRender;

      uint32_t _scanCount = 0;
      uint32_t SKIP_SCANS = 1;
      bool _saveScans = false;

};

#endif //MAP_SENSE_POINTCLOUDRECEIVER_H
