#include "NetworkManager.h"
#include "Log.h"

NetworkManager::NetworkManager(ApplicationState app, AppUtils *appUtils)
{
   this->appUtils = appUtils;
   if (app.STEREO_DRIVER)
   {
      this->camLeft = new cv::VideoCapture(0, cv::CAP_V4L2);
      this->camRight = new cv::VideoCapture(2, cv::CAP_V4L2);

      this->camLeft->set(cv::CAP_PROP_FRAME_WIDTH, 320);
      this->camLeft->set(cv::CAP_PROP_FRAME_HEIGHT, 240);
      this->camLeft->set(cv::CAP_PROP_FPS, 30);
      this->camLeft->set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
      this->camLeft->set(cv::CAP_PROP_AUTOFOCUS, 0);
      this->camLeft->set(cv::CAP_PROP_EXPOSURE, -4);
      this->camLeft->set(cv::CAP_PROP_MODE, cv::CAP_OPENCV_MJPEG);

      this->camRight->set(cv::CAP_PROP_FRAME_WIDTH, 320);
      this->camRight->set(cv::CAP_PROP_FRAME_HEIGHT, 240);
      this->camRight->set(cv::CAP_PROP_FPS, 30);
      this->camRight->set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
      this->camRight->set(cv::CAP_PROP_AUTOFOCUS, 0);
      this->camRight->set(cv::CAP_PROP_EXPOSURE, -4);
      this->camRight->set(cv::CAP_PROP_MODE, cv::CAP_OPENCV_MJPEG);
   }
}

void NetworkManager::SpinNode()
{
   ros::spinOnce();
}

void NetworkManager::InitNode(int argc, char **argv, ApplicationState& app)
{
   MS_INFO("Starting ROS Node");
   ros::init(argc, argv, "PlanarRegionPublisher");
   rosNode = new ros::NodeHandle();

   // ROSTopic Publishers
   planarRegionPub = rosNode->advertise<map_sense::RawGPUPlanarRegionList>("/mapsense/planar_regions", 3);
   slamPosePub = rosNode->advertise<sensor_msgs::PointCloud2>("/mapsense/slam/pose", 3);
   rawPlanesPub = rosNode->advertise<sensor_msgs::PointCloud2>("/slam/input/planes", 3);
   coloredCloudPub = rosNode->advertise<sensor_msgs::PointCloud2>("/mapsense/color/points", 2);

   // ROSTopic Subscribers
   std::string depthTopicName = app.DEPTH_ALIGNED ? app.L515_ALIGNED_DEPTH : app.L515_DEPTH;
   std::string depthInfoTopicName = app.DEPTH_ALIGNED ? app.L515_ALIGNED_DEPTH_INFO : app.L515_DEPTH_INFO;
   std::string colorTopicName = "/" + app.TOPIC_CAMERA_NAME + "/color/image_raw";

   std::string colorInfoTopicName = "/" + app.TOPIC_CAMERA_NAME + "/color/camera_info";

   app.L515_DEPTH = depthTopicName;
   app.L515_DEPTH_INFO = depthInfoTopicName;

   MS_INFO("L515 Depth Topic: {}", depthTopicName);
   MS_INFO("L515 Depth Info Topic: {}", depthInfoTopicName);

   AddReceiver(TopicInfo(depthTopicName, "sensor_msgs/Image"), TopicInfo(depthInfoTopicName, "sensor_msgs/CameraInfo"));
   AddReceiver(TopicInfo(app.L515_COLOR, "sensor_msgs/CompressedImage"));
   AddReceiver(TopicInfo(app.OUSTER_POINTS, "sensor_msgs/PointCloud2"));
   AddReceiver(TopicInfo(app.BLACKFLY_RIGHT_RAW, "sensor_msgs/Image"));

   AddReceiver(TopicInfo(app.ZED_LEFT_IMAGE_RAW, "sensor_msgs/Image"));
   AddReceiver(TopicInfo(app.ZED_RIGHT_IMAGE_RAW, "sensor_msgs/Image"));

   AddReceiver(TopicInfo(app.KITTI_LEFT_IMG_RECT, "sensor_msgs/CompressedImage"));
   AddReceiver(TopicInfo(app.KITTI_RIGHT_IMG_RECT, "sensor_msgs/CompressedImage"));
   AddReceiver(TopicInfo(app.KITTI_LIDAR_POINTS, "sensor_msgs/PointCloud2"));

   subMapSenseParams = rosNode->subscribe("/map/config", 8, &NetworkManager::MapsenseParamsCallback, this);
   subSLAMPose = rosNode->subscribe("/slam/output/pose", 2, &NetworkManager::SLAMPoseCallback, this);
   subSLAMPlanes = rosNode->subscribe("/slam/output/planes", 2, &NetworkManager::SLAMPlanesCallback, this);

   MS_INFO("Started ROS Node");
}

int NetworkManager::AddReceiver(TopicInfo data, TopicInfo info)
{
   MS_INFO("Adding Receiver: Topic:{}, Info:{}, Type:{}", data.name.c_str(), info.name.c_str(), data.datatype);
   ROS1TopicReceiver *receiver = nullptr;
   if (data.datatype == "sensor_msgs/Image")
      receiver = new ImageReceiver(rosNode, data.name, info.name, false);
   if (data.datatype == "sensor_msgs/CompressedImage")
      receiver = new ImageReceiver(rosNode, data.name, info.name, true);
   if (data.datatype == "sensor_msgs/PointCloud2")
      receiver = new PointCloudReceiver(rosNode, data.name, false);

   if (receiver != nullptr)
   {
      receiver->setAppUtils(this->appUtils);
      receivers[data.name] = receiver;
   } else
   {
      printf("Request to add receiver: %s\n", data.name.c_str());
   }
   MS_INFO("Receiver Added Successfully");
   return receivers.size() - 1;
}

void NetworkManager::ImGuiUpdate(ApplicationState& appState)
{
   if (ImGui::BeginTabItem("Network"))
   {
      std::vector<TopicInfo> topics = getROSTopicList();
      GetTopicSelection(topics, currentDataTopic);
      if (ImGui::Button("Add Receiver"))
         AddReceiver(currentDataTopic);
      for (std::pair<std::string, ROS1TopicReceiver *> receiver: receivers)
         receiver.second->ImGuiUpdate();

      ImGui::EndTabItem();
   }
}

void NetworkManager::SLAMPoseCallback(const sensor_msgs::PointCloud2ConstPtr& poseMsg)
{
   MS_INFO("Poses Received: {}", poseMsg->width, poseMsg->height);
   int totalBytes = poseMsg->width * poseMsg->height * poseMsg->point_step;
   std::vector<float> points(totalBytes / sizeof(float));
   memcpy(points.data(), poseMsg->data.data(), totalBytes);

//   _poses.clear();
   for(int i = 0; i<poseMsg->width * poseMsg->height; i++)
   {
      RigidBodyTransform transform;
      transform.SetQuaternionAndTranslation(Eigen::Quaterniond(points[i*8+6], points[i*8 + 3], points[i*8 + 4], points[i*8 + 5]),
                                            Eigen::Vector3d(points[i*8], points[i*8+1], points[i*8+2]));
      transform.SetID((int) points[i*8+7]);
      printf("Pose Received (%d): %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf\n", transform.GetID(), points[i*8], points[i*8+1], points[i*8+2],
             points[i*8+6], points[i*8 + 3], points[i*8 + 4], points[i*8 + 5]);
      transform.print();
      _poses.insert({(int) points[i*8+7], std::move(transform)});
   }
}

void NetworkManager::SLAMPlanesCallback(const sensor_msgs::PointCloud2ConstPtr& planeCloudMsg)
{
   MS_INFO("Planes Received: {} {} {}", planeCloudMsg->width, planeCloudMsg->height, planeCloudMsg->header.seq);
   int totalBytes = planeCloudMsg->width * planeCloudMsg->height * planeCloudMsg->point_step;
   std::vector<float> points(totalBytes / sizeof(float));
   memcpy(points.data(), planeCloudMsg->data.data(), totalBytes);

   planeSet.SetID((int)planeCloudMsg->header.seq + 1);
   for(int i = 0; i<planeCloudMsg->width * planeCloudMsg->height; i++)
   {
      planeSet.InsertPlane(Plane3D(points[i*5 ], points[i*5 + 1], points[i*5 + 2], points[i*5 + 3]),
                                   (int) points[i*5 + 4]);
   }
}

void NetworkManager::MapsenseParamsCallback(const map_sense::MapsenseConfiguration compressedMsg)
{
   paramsMessage = compressedMsg;
   paramsAvailable = true;
}

void NetworkManager::AcceptMapsenseConfiguration(ApplicationState& appState)
{
   if (paramsAvailable)
   {
      paramsAvailable = false;
      appState.MERGE_DISTANCE_THRESHOLD = paramsMessage.mergeDistanceThreshold;
      appState.MERGE_ANGULAR_THRESHOLD = paramsMessage.mergeAngularThreshold;
      appState.GAUSSIAN_SIGMA = (int) paramsMessage.gaussianSigma;
      appState.GAUSSIAN_SIZE = (int) paramsMessage.gaussianSize;
      appState.REGION_GROWTH_FACTOR = paramsMessage.regionGrowthFactor;
   }
}

std::vector<TopicInfo> NetworkManager::getROSTopicList()
{
   ros::master::V_TopicInfo topic_infos;
   ros::master::getTopics(topic_infos);
   std::vector<TopicInfo> names;
   for (int i = 0; i < topic_infos.size(); i++)
   {
      names.emplace_back(topic_infos[i]);
   }
   return names;
}

void NetworkManager::GetTopicSelection(std::vector<TopicInfo> topics, TopicInfo& currentTopic)
{
   if (ImGui::BeginCombo("ROS Topics", currentTopic.name.c_str()))
   {
      for (int n = 0; n < topics.size(); n++)
      {
         bool is_selected = (currentTopic.name.c_str() == topics[n].name.c_str());
         if (ImGui::Selectable(topics[n].name.c_str(), is_selected))
            currentTopic = topics[n];
         if (is_selected)
            ImGui::SetItemDefaultFocus();
      }
      ImGui::EndCombo();
   }
}

void NetworkManager::ReceiverUpdate(ApplicationState& app)
{
   for (std::pair<std::string, ROS1TopicReceiver *> receiver: receivers)
   {
      receiver.second->processMessage(app);
      receiver.second->render(app);
   }
}

void NetworkManager::publishSamplePose(int count)
{
   geometry_msgs::PoseStamped pose;
   pose.pose.position = geometry_msgs::Point();
   pose.pose.position.x = sin((double) count / (double) 100);
   pose.pose.position.y = cos((double) count / (double) 200);
   pose.pose.position.z = sin((double) count / (double) 300);

   pose.pose.orientation = geometry_msgs::Quaternion();

   this->slamPosePub.publish(pose);
}

//void NetworkManager::PublishColoredPointCloud(Clay::Ref<Clay::PointCloud> cloud)
//{
//
//   pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
//   pcl_cloud.width = cloud->GetSize();
//   pcl_cloud.height = 1;
//
//
//
//   for(int i = 0; i<cloud->GetSize(); i++)
//   {
//      pcl::PointXYZRGB p(cloud->GetColors()[i].r, cloud->GetColors()[i].g, cloud->GetColors()[i].b);
//      p.x = cloud->GetMesh()->_vertices[i*3 + 0];
//      p.y = cloud->GetMesh()->_vertices[i*3 + 1];
//      p.z = cloud->GetMesh()->_vertices[i*3 + 2];
//
////      uint8_t r = 255;
////      uint8_t g = 0;
////      uint8_t b = 0;
////      int32_t rgb = (r << 16) | (g << 8) | b;
//
//      pcl_cloud.points.push_back(p);
//   }
//
//
//   sensor_msgs::PointCloud2 msg;
////   msg.data = nullptr;
////   msg.fields = nullptr;
////   msg.header = nullptr;
////   msg.height = nullptr;
////   msg.width = nullptr;
////   msg.point_step = nullptr;
////   msg.row_step = nullptr;
//
//
//   pcl::toROSMsg(pcl_cloud, msg);
//   msg.header.frame_id = "ouster_frame";
//   coloredCloudPub.publish(msg);
//}

void NetworkManager::load_next_frame(cv::Mat& depth, cv::Mat& color, double& timestamp, ApplicationState& app)
{
   cv_bridge::CvImagePtr img_ptr_depth;
   cv_bridge::CvImagePtr img_ptr_color;
   cv_bridge::CvImagePtr img_ptr_color_compressed;
   MS_DEBUG("Process Data Callback");
   if (depthMessage != nullptr)
   {
      try
      {
         MS_DEBUG("Callback: Depth:{}", depthMessage->header.stamp.sec);
         img_ptr_depth = cv_bridge::toCvCopy(*depthMessage, image_encodings::TYPE_16UC1);
         depth = img_ptr_depth->image;
         timestamp = depthMessage.get()->header.stamp.toSec();

         for (int i = 0; i < app.DIVISION_FACTOR - 1; i++)
         {
            pyrDown(depth, depth);
         }

         MS_DEBUG("INPUT_DEPTH:({},{})", depth.rows, depth.cols);
      } catch (cv_bridge::Exception& e)
      {
         ROS_ERROR("Could not convert to _image! %s", e.what());
      }
   }
   if (colorMessage != nullptr)
   {
      try
      {
         MS_DEBUG("Callback: Color:{}", colorMessage->header.stamp.sec);
         img_ptr_color = cv_bridge::toCvCopy(*colorMessage, image_encodings::MONO8);
         color = img_ptr_color->image;

         for (int i = 0; i < app.DIVISION_FACTOR - 1; i++)
         {
            pyrDown(color, color);
         }
      } catch (cv_bridge::Exception& e)
      {
         ROS_ERROR("Could not convert to _image! %s", e.what());
      }
   } else if (colorCompressedMessage != nullptr)
   {
      try
      {
         MS_DEBUG("Callback: CompressedColor:{}", colorCompressedMessage->header.stamp.sec);
         //            img_ptr_color = cv_bridge::toCvCopy(*colorCompressedMessage, image_encodings::TYPE_8UC3);
         color = imdecode(cv::Mat(colorCompressedMessage->data), 1);
         for (int i = 0; i < app.DIVISION_FACTOR - 1; i++)
         {
            pyrDown(color, color);
         }
      } catch (cv_bridge::Exception& e)
      {
         ROS_ERROR("Could not convert compressedImage to _image! %s", e.what());
      }
   }
   if (depthCameraInfo != nullptr)
   {
      MS_DEBUG("DEPTH_SET: {}", depthCamInfoSet);
      if (!depthCamInfoSet)
      {
         depthCamInfoSet = true;
         app.DEPTH_INPUT_WIDTH = depthCameraInfo->width / app.DIVISION_FACTOR;
         app.DEPTH_INPUT_HEIGHT = depthCameraInfo->height / app.DIVISION_FACTOR;
         app.DEPTH_FX = depthCameraInfo->K[0] / app.DIVISION_FACTOR;
         app.DEPTH_FY = depthCameraInfo->K[4] / app.DIVISION_FACTOR;
         app.DEPTH_CX = depthCameraInfo->K[2] / app.DIVISION_FACTOR;
         app.DEPTH_CY = depthCameraInfo->K[5] / app.DIVISION_FACTOR;
         app.Update();
         MS_DEBUG("Process Callback: INPUT:({},{}), INFO:({}, {}, {}, {}, {}, {}) KERNEL:({},{}) PATCH:({},{})", app.DEPTH_INPUT_HEIGHT,
                   app.DEPTH_INPUT_WIDTH, app.SUB_W, app.SUB_H, app.DEPTH_FX, app.DEPTH_FY, app.DEPTH_CX, app.DEPTH_CY, app.SUB_H, app.SUB_W,
                   app.DEPTH_PATCH_HEIGHT, app.DEPTH_PATCH_WIDTH);
      }
      //        app.DEPTH_PATCH_HEIGHT = app.KERNEL_SLIDER_LEVEL;
      //        app.DEPTH_PATCH_WIDTH = app.KERNEL_SLIDER_LEVEL;
      //        app.SUB_H = (int) app.DEPTH_INPUT_HEIGHT / app.DEPTH_PATCH_HEIGHT;
      //        app.SUB_W = (int) app.DEPTH_INPUT_WIDTH / app.DEPTH_PATCH_WIDTH;


      MS_DEBUG("INPUT:({},{}), INFO:({}, {}, {}, {}, {}, {}) KERNEL:({},{}) PATCH:({},{})", app.DEPTH_INPUT_HEIGHT, app.DEPTH_INPUT_WIDTH, app.SUB_W,
                app.SUB_H, app.DEPTH_FX, app.DEPTH_FY, app.DEPTH_CX, app.DEPTH_CY, app.SUB_H, app.SUB_W, app.DEPTH_PATCH_HEIGHT, app.DEPTH_PATCH_WIDTH);
   }
   MS_DEBUG("Data Frame Loaded");
}

void NetworkManager::PublishPlanes(const std::vector<std::shared_ptr<PlanarRegion>>& regions, int poseId)
{
   std::vector<float> points;

   int count = 0;
   for (auto region: regions)
   {
      if(region->getId() >= 0)
      {
         count++;
         points.push_back((float)poseId);
         points.push_back(region->GetCenter().x());
         points.push_back(region->GetCenter().y());
         points.push_back(region->GetCenter().z());
         points.push_back(region->GetNormal().x());
         points.push_back(region->GetNormal().y());
         points.push_back(region->GetNormal().z());
         points.push_back((float)region->getId());

         MS_INFO("Publishing Plane ({}): {} {} {} {} {} {}", region->getId(), region->GetCenter().x(), region->GetCenter().y(), region->GetCenter().z(),
                       region->GetNormal().x(), region->GetNormal().y(), region->GetNormal().z());
      }
   }

   std::vector<unsigned char> data(points.size() * sizeof(float));
   memcpy(data.data(), points.data(), data.size());
   MS_INFO("Data: {} Points:{}", data.size(), points.size());

   sensor_msgs::PointCloud2 planeSet;
   planeSet.data = data;
   planeSet.width = count;
   planeSet.height = 1;
   planeSet.row_step = 2;
   planeSet.point_step = 4 * 8;
   rawPlanesPub.publish(planeSet);
}

void NetworkManager::PublishPoseStamped(RigidBodyTransform odometry, int poseId)
{
   Eigen::Quaterniond quaternion = odometry.GetQuaternion();
   Eigen::Vector3d position = odometry.GetTranslation();

   sensor_msgs::PointCloud2 poseSet;
   poseSet.width = 1;
   poseSet.height = 1;
   poseSet.row_step = 1;
   poseSet.point_step = 4 * 8;

   std::vector<float> buffer;

   buffer.push_back(position.x());
   buffer.push_back(position.y());
   buffer.push_back(position.z());
   buffer.push_back(quaternion.x());
   buffer.push_back(quaternion.y());
   buffer.push_back(quaternion.z());
   buffer.push_back(quaternion.w());
   buffer.push_back(poseId);

   MS_INFO("Publishing Pose ({}): {} {} {} {} {} {} {}", poseId, position.x(),
                 position.y(), position.z(), quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());

   std::vector<unsigned char> data(buffer.size() * sizeof(float));
   memcpy(data.data(), buffer.data(), data.size());
   poseSet.data = data;

   slamPosePub.publish(poseSet);
}

void NetworkManager::load_next_stereo_frame(cv::Mat& left, cv::Mat& right, ApplicationState& app)
{
   camLeft->read(left);
   camRight->read(right);
}

void NetworkManager::depthCallback(const ImageConstPtr& depthMsg)
{
   MS_DEBUG("Depth Callback: {}", depthMsg->header.seq);
   depthMessage = depthMsg;
}

void NetworkManager::colorCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& infoMsg)
{
   colorCameraInfo = infoMsg;
   MS_DEBUG("COLOR_CAM_INFO CALLBACK");
}

void NetworkManager::depthCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& infoMsg)
{
   depthCameraInfo = infoMsg;
   MS_DEBUG("DEPTH_CAM_INFO CALLBACK");
}

void NetworkManager::colorCompressedCallback(const sensor_msgs::CompressedImageConstPtr& compressedMsg)
{
   colorCompressedMessage = compressedMsg;
   MS_DEBUG("COLOR CALLBACK");
}

void NetworkManager::colorCallback(const sensor_msgs::ImageConstPtr& colorMsg, std::string name)
{
   colorMessage = colorMsg;
   MS_DEBUG("COLOR CALLBACK");
}
