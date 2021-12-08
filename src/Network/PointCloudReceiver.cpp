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
   if (_available && _renderEnabled )
   {
      uint32_t numPoints = 0;
      _cloudToRender->Reset();


      _available = false;
   }
//   if(_renderEnabled) _cloudToRender->Upload();
}

void PointCloudReceiver::ImGuiUpdate()
{
   ImGui::NewLine();
   ImGui::Checkbox("Render", &_renderEnabled);
   ImGui::Checkbox("Save Scans", &_saveScans);
}

void PointCloudReceiver::cloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloudMsg)
{
   CLAY_LOG_INFO("PointCloudCallback: {} {} {}", cloudMsg->header.frame_id, cloudMsg->height, cloudMsg->width);
   if (!_available)
   {
      _cloudMessage = cloudMsg;
      _available = true;

      Clay::Ref<Clay::PointCloud> cloud;
      cloud = std::make_shared<Clay::PointCloud>(glm::vec4(0.3, 0.4, 0.5, 1.0), nullptr);

      int count = 0;
      for(const pcl::PointXYZ& pt : _cloudMessage->points)
      {
         count++;
         if(!(pt.x == 0 && pt.y == 0 && pt.z == 0) && count % 4 == 0)
         {
            cloud->InsertVertex(-pt.y, pt.z, -pt.x);
         }
      }
      _clouds.push_back(std::move(cloud));
      CLAY_LOG_INFO("Clouds: {}", _clouds.size());
   }
   if(_saveScans) {
      if(_scanCount % _skipScans == 0)DataManager::WriteScanPoints(cloudMsg, _scanCount);
      _scanCount++;
   }
}
