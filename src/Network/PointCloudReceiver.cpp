//
// Created by quantum on 9/23/21.
//

#include "PointCloudReceiver.h"
//#include <xtensor/xarray.hpp>
//#include <xtensor/xadapt.hpp>
//#include <xtensor/xio.hpp>
//#include <xtensor/xview.hpp>
//#include <xtensor/xmasked_view.hpp>
//#include <xtensor/xindex_view.hpp>
//#include <xtensor/xdynamic_view.hpp>
//#include <xtensor-blas/xlinalg.hpp>

PointCloudReceiver::PointCloudReceiver(NodeHandle *nh, std::string cloudTopic, bool compressed)
{
   CLAY_LOG_INFO("PointCloudReceiver Created!");
   topicName = cloudTopic;
   _cloudSubscriber = new Subscriber();
   _cloudToRender = std::make_shared<Clay::PointCloud>(glm::vec4(0.8f, 0.3f, 0.3f, 1.0f), nullptr);

//   *(this->_cloudSubscriber) = nh->subscribe(cloudTopic, 2, &PointCloudReceiver::cloudCallback, this);

   *(this->_cloudSubscriber) = nh->subscribe(cloudTopic, 2, &PointCloudReceiver::PointCloud2Callback, this);


}

void PointCloudReceiver::PointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
      if (!_available)
      {
         _msg = cloudMsg;
         _available = true;

         Clay::Ref<Clay::PointCloud> cloud;
         cloud = std::make_shared<Clay::PointCloud>(glm::vec4(0.3, 0.4, 0.5, 1.0), nullptr);

         int count = 0;
         for(int i = 0; i < _msg->width * _msg->height; i++)
         {
            count++;
            Eigen::Vector3f pt(_msg->data[i*3], _msg->data[i*3+1], _msg->data[i*3+2]);
            if(!(pt.x() == 0 && pt.y() == 0 && pt.z() == 0))
            {
               cloud->InsertVertex(pt.x(), pt.y(), pt.z());
            }
         }
         _clouds.push_back(std::move(cloud));
         CLAY_LOG_INFO("Clouds: {} {}", topicName, _clouds.size());
         if(_clouds.size() > 2) _clouds.erase(_clouds.end());
      }
//      if(_saveScans) {
//         if(_scanCount % _skipScans == 0)DataManager::WriteScanPoints(cloudMsg, _scanCount);
//         _scanCount++;
//      }
}

void PointCloudReceiver::processMessage(ApplicationState& app)
{
   ROS_DEBUG("PointCloud Processing!");
}

void PointCloudReceiver::getData(cv::Mat& image, ApplicationState& app, double& timestamp)
{

}

void PointCloudReceiver::render(ApplicationState& app)
{
   if (_available && _renderEnabled )
   {
      uint32_t numPoints = 0;
      _cloudToRender->Reset();

      _available = false;
   }
}

void PointCloudReceiver::ImGuiUpdate()
{
   ImGui::Checkbox("Render", &_renderEnabled);
   ImGui::Checkbox("Save Scans", &_saveScans);
}

//void PointCloudReceiver::cloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloudMsg)
//{
////   CLAY_LOG_INFO("PointCloudCallback: {} {} {}", cloudMsg->header.frame_id, cloudMsg->height, cloudMsg->width);
//   if (!_available)
//   {
//      _cloudMessage = cloudMsg;
//      _available = true;
//
//      Clay::Ref<Clay::PointCloud> cloud;
//      cloud = std::make_shared<Clay::PointCloud>(glm::vec4(0.3, 0.4, 0.5, 1.0), nullptr);
//
//      int count = 0;
//      for(const pcl::PointXYZ& pt : _cloudMessage->points)
//      {
//         count++;
//         if(!(pt.x == 0 && pt.y == 0 && pt.z == 0))
//         {
//            cloud->InsertVertex(pt.x, pt.y, pt.z);
//         }
//      }
//      _clouds.push_back(std::move(cloud));
////      CLAY_LOG_INFO("Clouds: {} {}", topicName, _clouds.size());
//      if(_clouds.size() > 2) _clouds.erase(_clouds.end());
//   }
//   if(_saveScans) {
//      if(_scanCount % _skipScans == 0)DataManager::WriteScanPoints(cloudMsg, _scanCount);
//      _scanCount++;
//   }
//}

void PointCloudReceiver::ColorPointsByImage(Clay::Ref<Clay::PointCloud> cloud, cv::Mat image, float ousterPitch)
{
//   /* Transform:  [   0.006928    -0.99997  -0.0027578   -0.024577]
//                [  -0.001163   0.0027498          -1   -0.061272]
//                [    0.99998   0.0069311  -0.0011439     -0.3321]
//                [          0           0           0           1]
//
//      Projection: [[     707.05           0      604.08           0]
//                  [          0      707.05      180.51           0]
//                  [          0           0           1           0]]
//    */
//
//
//    CLAY_LOG_INFO("Color Pointcloud Update");
//
//   // Initialize Projection and Transformation Matrices
//   // BlackFly Intrinsics Right: 499.3716197917922, 0.0, 1043.8826790137316, 0.0, 506.42956667285574, 572.2558510618412, 0.0, 0.0, 1.0
//   xt::xarray<float> ProjMat = { {     499.3716197917922,           0,      1043.8826790137316,           0},
//                                 {          0,      506.42956667285574,      572.2558510618412,           0},
//                                 {          0,           0,           1,           0}};
//   xt::xarray<float> TransformToCam =  {{    0.006928,    -0.99997,    -0.0027578,   -0.024577},
//                                       {     -0.001163,   0.0027498,           -1,   -0.061272},
//                                       {     0.99998,     0.0069311,   -0.0011439,     -0.3321},
//                                       {     0,                   0,            0,           1}};
//   float pitch = ousterPitch/180 * M_PI;
//
//   xt::xarray<float> TransformPitchDown =  {{    cos(pitch),   0,    sin(pitch),  0},
//                                           {     0,               1,    0,             0},
//                                           {     -sin(pitch),  0,    cos(pitch),  0},
//                                           {     0.0,             0,    0.0,           1}};
//
//   // Create XTensor array for OpenCV Image
//   size_t size = image.total();
//   size_t channels = image.channels();
//   xt::xarray<float> xImage = xt::adapt((uint8_t *) image.data, image.cols * image.rows * 3,
//         xt::no_ownership(), std::vector<std::size_t>{(uint32_t)image.rows, (uint32_t)image.cols, (uint32_t)image.channels()});
//
//    CLAY_LOG_INFO("xImage: {} {}", xImage.shape()[0], xImage.shape()[1], xImage.shape()[2]);
//
//   const uint32_t length = cloud->GetSize();
//   auto ones = xt::ones<float>({1, (int)length});
//
//   // Create XTensor array for 3d points
//   std::vector<std::size_t> shape = { length, 3 };
//   auto points3D = xt::transpose(xt::adapt(cloud->GetMesh()->_vertices, shape));
//   auto hPoints3D = xt::vstack(xt::xtuple(points3D, ones));
//   auto pitchHPoints3D = xt::linalg::dot(TransformPitchDown, hPoints3D);
//
//   // Remove all points with X-forward negative.
//   auto xPositiveIndices = xt::from_indices(xt::argwhere(xt::view(pitchHPoints3D, 0) > 0));
//   auto hPoints3DX = xt::view(pitchHPoints3D, xt::all(), xt::keep(xPositiveIndices));
//
//   // Matrix Multiply Projection and Transform Matrices
//   auto hCamPoints = xt::linalg::dot(xt::linalg::dot(ProjMat, TransformToCam), hPoints3DX);
//
//   // Normalize Homogeneous Image Points
//   xt::view(hCamPoints, 0, xt::all()) = xt::view(hCamPoints, 0, xt::all()) / xt::view(hCamPoints, 2, xt::all());
//   xt::view(hCamPoints, 1, xt::all()) = xt::view(hCamPoints, 1, xt::all()) / xt::view(hCamPoints, 2, xt::all());
//
//   // Select all Image Points and 3D points where Z was positive after projection.
//   auto zCamPositiveIndices = xt::from_indices(xt::argwhere(xt::view(hCamPoints, 2) > 0));
//   auto hCamPointsZ = xt::view(hCamPoints, xt::all(), xt::keep(zCamPositiveIndices));
//   auto hCamPoints3DXZ = xt::view(hPoints3DX, xt::all(), xt::keep(zCamPositiveIndices));
//
//   // Select all points for which the projection falls within Image frame bounds.
//   auto imgIndices = xt::from_indices(xt::argwhere(xt::view(hCamPointsZ, 0) > 0 && xt::view(hCamPointsZ, 0) < image.cols &&
//                                 xt::view(hCamPointsZ, 1) > 0 && xt::view(hCamPointsZ, 1) < image.rows));
//   auto camPointsImg = xt::view(hCamPointsZ, xt::all(), xt::keep(imgIndices));
//   camPointsImg = xt::cast<int>(camPointsImg);
//   auto hCamPoints3DXZImg = xt::view(hCamPoints3DXZ, xt::all(), xt::keep(imgIndices));
//
//   std::cout << camPointsImg << std::endl;
//   std::cout << hCamPoints3DXZImg << std::endl;
//
//    CLAY_LOG_INFO("HCamPoints3D: {} {}", hCamPoints3DXZImg.shape()[0], hCamPoints3DXZImg.shape()[1]);
//
//   int count = 0;
//   cloud->Reset();
//   for(int i = 0; i<hCamPoints3DXZImg.shape()[1]; i++)
//   {
//      count++;
//      if(!(hCamPoints3DXZImg(0,i) == 0 && hCamPoints3DXZImg(1,i) == 0 && hCamPoints3DXZImg(2,i) == 0) && count % 2 == 0)
//      {
////         cloud->InsertVertex(-hCamPoints3DXZImg(1,i), hCamPoints3DXZImg(2,i), -hCamPoints3DXZImg(0,i));
//          cloud->InsertVertex(hCamPoints3DXZImg(0,i), hCamPoints3DXZImg(1,i), hCamPoints3DXZImg(2,i));
//         cloud->InsertIndex(i);
//
//         cv::Vec3b color = image.at<cv::Vec3b>(camPointsImg(0,i),camPointsImg(1,i));
//         cloud->InsertColor({color[0], color[1], color[2], 1});
//      }
//   }
//
//   CLAY_LOG_INFO("Points3D Shape: {} {}", camPointsImg.shape()[0], camPointsImg.shape()[1]);
//
//   /*
//
//   auto indicesX = xt::cast<size_t>(xt::view(camPointsImg, 0));
//   auto indicesY = xt::cast<size_t>(xt::view(camPointsImg, 1));
//
//   CLAY_LOG_INFO("Image Shape: {} {}", xImage.shape()[0], xImage.shape()[1]);
//
//   xt::xarray<int> colors = xt::view(xImage, xt::keep(indicesY), xt::keep(indicesX), xt::all());
//
//   CLAY_LOG_INFO("CamPoints Shape: {} {} {}", colors.shape()[0], colors.shape()[1], colors.shape()[2]);
//
//    */
//
//   for(int i = 0; i<camPointsImg.shape()[1]; i++)
//   {
//      // cv2.circle(im0, (int(u[0, i]), int(v[0, i])), 1, (int(z[0, i] / 20 * 255), 200, int(z[0, i] / 20 * 255)),-1)
//      cv::circle(image, cv::Point(camPointsImg(0,i), camPointsImg(1,i)), 2,
//                 cv::Scalar((int) (camPointsImg(2,i) / 10 * 255),(int) (camPointsImg(2,i) / 10 * 255),200), -1);
//   }

//      cloud = get_rotation_y(- self.pitch / 180 * np.pi) @ np.insert(cloud, 3, 1, axis=0)
//
//   # Return Original, Non-Homogeneous Frustrum(ized) and Projected 2D point sets.
//      return xyz, cloud[:3, :], cam_points
}
