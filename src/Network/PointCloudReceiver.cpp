//
// Created by quantum on 9/23/21.
//

#include "PointCloudReceiver.h"
#include <xtensor/xarray.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xio.hpp>

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
//   CLAY_LOG_INFO("PointCloudCallback: {} {} {}", cloudMsg->header.frame_id, cloudMsg->height, cloudMsg->width);
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
//      CLAY_LOG_INFO("Clouds: {}", _clouds.size());
   }
   if(_saveScans) {
      if(_scanCount % _skipScans == 0)DataManager::WriteScanPoints(cloudMsg, _scanCount);
      _scanCount++;
   }
}

void PointCloudReceiver::ColorPointsByImage(Clay::Ref<Clay::PointCloud> cloud, cv::Mat image)
{
   const uint32_t length = cloud->GetSize();
//   auto points = xt::zeros<float>({3, length});
   auto ones = xt::ones<float>({1, (int)length});

   std::vector<double> v = {1., 2., 3., 4., 5., 6. };
   std::vector<std::size_t> shape = { 3, length };
   auto points = xt::adapt(cloud->GetMesh()->_vertices, shape);


//   # Convert to Homogeneous
//      vel_points = np.insert(xyz, 3, 1, axis=1).T
   auto homo_points = xt::vstack(xt::xtuple(points, ones));

   std::cout << homo_points << std::endl;

//
//   # Remove all points with X-forward negative.
//      vel_points = np.delete(vel_points, np.where(vel_points[0, :] < 0), axis=1)
//
//   # Create non-homogeneous copy.
//      cloud = vel_points.copy()
//      cloud = np.delete(cloud, 3, axis=0)
//
//   # Project onto Camera
//      cam_points = ProjMat @ TransformToCam @ vel_points
//
//   # Remove all 3D points from 'cloud' behind the Camera (Z-forward negative).
//      cloud = np.delete(cloud, np.where(cam_points[2, :] < 0), axis=1)
//
//   # Remove all 2D projected with Z-forward negative.
//      cam_points = np.delete(cam_points, np.where(cam_points[2, :] < 0), axis=1)
//
//   # Normalize by the Z-coordinate of Projected points.
//      cam_points[:2] /= cam_points[2, :]
//      u, v, z = cam_points
//
//   # Remove all points outside that were projected outside the image frame.
//      u_out = np.logical_or(u < 0, u > width)
//      v_out = np.logical_or(v < 0, v > height)
//      outlier = np.logical_or(u_out, v_out)
//      cloud = np.delete(cloud, np.where(outlier), axis=1)
//      cam_points = np.delete(cam_points, np.where(outlier), axis=1)
//
//      cloud = get_rotation_y(- self.pitch / 180 * np.pi) @ np.insert(cloud, 3, 1, axis=0)
//
//   # Return Original, Non-Homogeneous Frustrum(ized) and Projected 2D point sets.
//      return xyz, cloud[:3, :], cam_points
}
