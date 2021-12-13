//
// Created by quantum on 9/23/21.
//

#include "PointCloudReceiver.h"
#include <xtensor/xarray.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xmasked_view.hpp>
#include <xtensor/xindex_view.hpp>
#include <xtensor/xdynamic_view.hpp>
#include <xtensor-blas/xlinalg.hpp>

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
            cloud->InsertVertex(pt.x, pt.y, pt.z);
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
   /* Transform:  [   0.006928    -0.99997  -0.0027578   -0.024577]
                [  -0.001163   0.0027498          -1   -0.061272]
                [    0.99998   0.0069311  -0.0011439     -0.3321]
                [          0           0           0           1]

      Projection: [[     707.05           0      604.08           0]
                  [          0      707.05      180.51           0]
                  [          0           0           1           0]]
    */

   // Initialize Projection and Transformation Matrices
   xt::xarray<float> ProjMat = { {     707.05,           0,      604.08,           0},
                                 {          0,      707.05,      180.51,           0},
                                 {          0,           0,           1,           0}};
   xt::xarray<float> TransformToCam =  {{    0.006928,    -0.99997,    -0.0027578,   -0.024577},
                                       {     -0.001163,   0.0027498,           -1,   -0.061272},
                                       {     0.99998,     0.0069311,   -0.0011439,     -0.3321},
                                       {     0,                   0,            0,           1}};

   std::vector<float> testPoints = {1., 2., 2.3, 3.2, 0.1, 3.,1., 3., 4.3, 6.2, 7.1, 8.,1., 9., 5.3, 3.1, 4.1, 7.8};
   std::vector<std::size_t> s = { 6, 3 };
   auto a1 = xt::transpose(xt::adapt(testPoints, s));

   CLAY_LOG_INFO("TestPoints: ");
   std::cout << a1 << std::endl;

   // Create XTensor array for OpenCV Image
   xt::xarray<float> xImage = xt::adapt(
         (float*) image.data, image.cols * image.rows,
         xt::no_ownership(), std::vector<std::size_t> {(uint32_t)image.rows, (uint32_t)image.cols});

   const uint32_t length = cloud->GetSize();
   auto ones = xt::ones<float>({1, (int)length});

   // Create XTensor array for 3d points
   std::vector<std::size_t> shape = { length, 3 };
   auto points3D = xt::transpose(xt::adapt(cloud->GetMesh()->_vertices, shape));
   auto hPoints3D = xt::vstack(xt::xtuple(points3D, ones));

//   CLAY_LOG_INFO("Points3D:");
//   std::cout << hPoints3D << std::endl;

   // Remove all points with X-forward negative.
   auto xPositiveIndices = xt::from_indices(xt::argwhere(xt::view(hPoints3D, 0) > 0));
   auto hPoints3DX = xt::view(hPoints3D, xt::all(), xt::keep(xPositiveIndices));

   // Matrix Multiply Projection and Transform Matrices
   auto hCamPoints = xt::linalg::dot(xt::linalg::dot(ProjMat, TransformToCam), hPoints3DX);

   // Normalize Homogeneous Image Points
   xt::view(hCamPoints, 0, xt::all()) = xt::view(hCamPoints, 0, xt::all()) / xt::view(hCamPoints, 2, xt::all());
   xt::view(hCamPoints, 1, xt::all()) = xt::view(hCamPoints, 1, xt::all()) / xt::view(hCamPoints, 2, xt::all());

   // Select all Image Points and 3D points where Z was positive after projection.
   auto zCamPositiveIndices = xt::from_indices(xt::argwhere(xt::view(hCamPoints, 2) > 0));
   auto hCamPointsZ = xt::view(hCamPoints, xt::all(), xt::keep(zCamPositiveIndices));
   auto hCamPoints3D = xt::view(hPoints3D, xt::all(), xt::keep(zCamPositiveIndices));

   // Select all points for which the projection falls within Image frame bounds.
   auto imgIndices = xt::from_indices(xt::argwhere(xt::view(hCamPointsZ, 0) > 0 && xt::view(hCamPointsZ, 0) < image.cols &&
                                 xt::view(hCamPointsZ, 1) > 0 && xt::view(hCamPointsZ, 1) < image.rows));
   auto camPointsImg = xt::view(hCamPointsZ, xt::all(), xt::keep(imgIndices));
   camPointsImg = xt::cast<int>(camPointsImg);
   auto hCamPoints3DImg = xt::view(hCamPoints3D, xt::all(), xt::keep(imgIndices));

   int count = 0;
   cloud->Reset();
   for(int i = 0; i<hCamPoints3DImg.shape()[1]; i++)
   {
      count++;
      if(!(hCamPoints3DImg(0,i) == 0 && hCamPoints3DImg(1,i) == 0 && hCamPoints3DImg(2,i) == 0))
      {
         cloud->InsertVertex(-hCamPoints3DImg(1,i), hCamPoints3DImg(2,i), -hCamPoints3DImg(0,i));
         cloud->InsertIndex(i);
      }
   }

   CLAY_LOG_INFO("Points3D Shape: {} {}", hCamPoints3DImg.shape()[0], hCamPoints3DImg.shape()[1]);
   CLAY_LOG_INFO("CamPoints Shape: {} {}", camPointsImg.shape()[0], camPointsImg.shape()[1]);

//   xt::xarray<int> colors = xt::view(xImage, xt::view(camPointsImg, 0), xt::view(camPointsImg, 0), xt::all());


   std::cout << camPointsImg << std::endl;
   std::cout << hCamPoints3DImg << std::endl;

   for(int i = 0; i<camPointsImg.shape()[1]; i++)
   {
      cv::circle(image, cv::Point(camPointsImg(0,i), camPointsImg(1,i)), 2, cv::Scalar(255,255,0), -1);
   }

//      cloud = get_rotation_y(- self.pitch / 180 * np.pi) @ np.insert(cloud, 3, 1, axis=0)
//
//   # Return Original, Non-Homogeneous Frustrum(ized) and Projected 2D point sets.
//      return xyz, cloud[:3, :], cam_points
}
