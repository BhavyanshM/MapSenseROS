#include "NetworkManager.h"

void NetworkManager::depthCallback(const ImageConstPtr& depthMsg)
{
   ROS_DEBUG("Depth Callback", depthMsg->header.seq);
   depthMessage = depthMsg;
}

void NetworkManager::colorCallback(const sensor_msgs::ImageConstPtr& colorMsg)
{
   colorMessage = colorMsg;
   ROS_DEBUG("COLOR CALLBACK");
}

void NetworkManager::colorCompressedCallback(const sensor_msgs::CompressedImageConstPtr& compressedMsg)
{
   colorCompressedMessage = compressedMsg;
   ROS_DEBUG("COLOR CALLBACK");
}

void NetworkManager::depthCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& infoMsg)
{
   depthCameraInfo = infoMsg;
   ROS_DEBUG("DEPTH_CAM_INFO CALLBACK");
}

void NetworkManager::colorCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& infoMsg)
{
   colorCameraInfo = infoMsg;
   ROS_DEBUG("COLOR_CAM_INFO CALLBACK");
}

void NetworkManager::mapSenseParamsCallback(const map_sense::MapsenseConfiguration msg)
{
   paramsMessage = msg;
   ROS_DEBUG("COLOR CALLBACK");
}

void NetworkManager::load_next_frame(Mat& depth, Mat& color, ApplicationState& app)
{
   cv_bridge::CvImagePtr img_ptr_depth;
   cv_bridge::CvImagePtr img_ptr_color;
   cv_bridge::CvImagePtr img_ptr_color_compressed;
   ROS_DEBUG("Process Data Callback");
   if (depthMessage != nullptr)
   {
      try
      {
         ROS_DEBUG("Callback: Depth:%d", depthMessage->header.stamp.sec);
         img_ptr_depth = cv_bridge::toCvCopy(*depthMessage, image_encodings::TYPE_16UC1);
         depth = img_ptr_depth->image;
         for (int i = 0; i < app.DIVISION_FACTOR - 1; i++)
         {
            pyrDown(depth, depth);
         }
         ROS_DEBUG("INPUT_DEPTH:(%d,%d)", depth.rows, depth.cols);
      } catch (cv_bridge::Exception& e)
      {
         ROS_ERROR("Could not convert to image! %s", e.what());
      }
   }
   if (colorMessage != nullptr)
   {
      try
      {
         ROS_DEBUG("Callback: Color:%d", colorMessage->header.stamp.sec);
         img_ptr_color = cv_bridge::toCvCopy(*colorMessage, image_encodings::TYPE_8UC3);
         color = img_ptr_color->image;
         for (int i = 0; i < app.DIVISION_FACTOR - 1; i++)
         {
            pyrDown(color, color);
         }
      } catch (cv_bridge::Exception& e)
      {
         ROS_ERROR("Could not convert to image! %s", e.what());
      }
   } else if (colorCompressedMessage != nullptr)
   {
      try
      {
         ROS_DEBUG("Callback: CompressedColor:%d", colorCompressedMessage->header.stamp.sec);
         //            img_ptr_color = cv_bridge::toCvCopy(*colorCompressedMessage, image_encodings::TYPE_8UC3);
         color = imdecode(cv::Mat(colorCompressedMessage->data), 1);
         for (int i = 0; i < app.DIVISION_FACTOR - 1; i++)
         {
            pyrDown(color, color);
         }
      } catch (cv_bridge::Exception& e)
      {
         ROS_ERROR("Could not convert compressedImage to image! %s", e.what());
      }
   }
   if (depthCameraInfo != nullptr)
   {
      ROS_DEBUG("DEPTH_SET:", depthCamInfoSet);
      //      if (!depthCamInfoSet)
      {
         depthCamInfoSet = true;
         app.INPUT_WIDTH = depthCameraInfo->width / app.DIVISION_FACTOR;
         app.INPUT_HEIGHT = depthCameraInfo->height / app.DIVISION_FACTOR;
         app.DEPTH_FX = depthCameraInfo->K[0] / app.DIVISION_FACTOR;
         app.DEPTH_FY = depthCameraInfo->K[4] / app.DIVISION_FACTOR;
         app.DEPTH_CX = depthCameraInfo->K[2] / app.DIVISION_FACTOR;
         app.DEPTH_CY = depthCameraInfo->K[5] / app.DIVISION_FACTOR;
         app.update();
         ROS_DEBUG("Process Callback: INPUT:(%d,%d), INFO:(%d, %d, %.2f, %.2f, %.2f, %.2f) KERNEL:(%d,%d) PATCH:(%d,%d)", app.INPUT_HEIGHT, app.INPUT_WIDTH,
                   app.SUB_W, app.SUB_H, app.DEPTH_FX, app.DEPTH_FY, app.DEPTH_CX, app.DEPTH_CY, app.SUB_H, app.SUB_W, app.PATCH_HEIGHT, app.PATCH_WIDTH);
      }
      //        app.PATCH_HEIGHT = app.KERNEL_SLIDER_LEVEL;
      //        app.PATCH_WIDTH = app.KERNEL_SLIDER_LEVEL;
      //        app.SUB_H = (int) app.INPUT_HEIGHT / app.PATCH_HEIGHT;
      //        app.SUB_W = (int) app.INPUT_WIDTH / app.PATCH_WIDTH;


      ROS_DEBUG("INPUT:(%d,%d), INFO:(%d, %d, %.2f, %.2f, %.2f, %.2f) KERNEL:(%d,%d) PATCH:(%d,%d)", app.INPUT_HEIGHT, app.INPUT_WIDTH, app.SUB_W, app.SUB_H,
                app.DEPTH_FX, app.DEPTH_FY, app.DEPTH_CX, app.DEPTH_CY, app.SUB_H, app.SUB_W, app.PATCH_HEIGHT, app.PATCH_WIDTH);
   }
   ROS_DEBUG("Data Frame Loaded");
}

void NetworkManager::init_ros_node(int argc, char **argv)
{
   ROS_INFO("Starting ROS Node");
   init(argc, argv, "PlanarRegionPublisher");
   nh = new NodeHandle();

   planarRegionPub = nh->advertise<map_sense::RawGPUPlanarRegionList>("/map/regions/test", 3);

   subDepth = nh->subscribe("/camera/depth/image_rect_raw", 3, &NetworkManager::depthCallback, this);
   subColor = nh->subscribe("/camera/color/image_raw", 3, &NetworkManager::colorCallback, this);
   subColorCompressed = nh->subscribe("/camera/color/image_raw/compressed", 3, &NetworkManager::colorCompressedCallback, this);
   subDepthCamInfo = nh->subscribe("/camera/depth/camera_info", 2, &NetworkManager::depthCameraInfoCallback, this);
   subColorCamInfo = nh->subscribe("/camera/color/camera_info", 2, &NetworkManager::colorCameraInfoCallback, this);
   //    subMapSenseParams = nh->subscribe("/map_sense/params", 8, &NetworkManager::colorCompressedCallback, this);

   ROS_INFO("Started ROS Node");
}

void NetworkManager::spin_ros_node()
{
   ROS_DEBUG("SpinOnce");
   spinOnce();
}

void NetworkManager::load_sample_depth(String filename, Mat& depth)
{
   depth = imread(ros::package::getPath("map_sense") + filename, IMREAD_ANYDEPTH);
}

void NetworkManager::load_sample_color(String filename, Mat& color)
{
   color = imread(ros::package::getPath("map_sense") + filename, IMREAD_COLOR);
}

void NetworkManager::get_sample_depth(Mat depth, float mean, float stddev)
{
   std::default_random_engine generator;
   std::normal_distribution<double> distribution(mean, stddev);
   for (int i = 0; i < depth.cols; i++)
   {
      for (int j = 0; j < depth.rows; j++)
      {
         float d = 10.04;
         d += distribution(generator);
         if (160 < i && i < 350 && 200 < j && j < 390)
         {
            // d = 0.008 * i + 0.014 * j;
            depth.at<short>(j, i) = (d - 2.0f) * 1000;
         } else
         {
            depth.at<short>(j, i) = d * 1000;
         }
      }
   }
}

void NetworkManager::get_sample_color(Mat color)
{
   for (int i = 0; i < color.rows; i++)
   {
      for (int j = 0; j < color.cols; j++)
      {
         color.at<Vec3b>(i, j) = Vec3b(0, 0, 255);
      }
   }
}