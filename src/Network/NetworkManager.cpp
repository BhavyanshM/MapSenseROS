#include "NetworkManager.h"

NetworkManager::NetworkManager(ApplicationState app, AppUtils *appUtils)
{
   this->appUtils = appUtils;
   if (app.STEREO_DRIVER)
   {
      this->camLeft = new VideoCapture(0, CAP_V4L2);
      this->camRight = new VideoCapture(2, CAP_V4L2);

      this->camLeft->set(CAP_PROP_FRAME_WIDTH, 320);
      this->camLeft->set(CAP_PROP_FRAME_HEIGHT, 240);
      this->camLeft->set(CAP_PROP_FPS, 30);
      this->camLeft->set(CAP_PROP_AUTO_EXPOSURE, 0.25);
      this->camLeft->set(CAP_PROP_AUTOFOCUS, 0);
      this->camLeft->set(CAP_PROP_EXPOSURE, -4);
      this->camLeft->set(CAP_PROP_MODE, CAP_OPENCV_MJPEG);

      this->camRight->set(CAP_PROP_FRAME_WIDTH, 320);
      this->camRight->set(CAP_PROP_FRAME_HEIGHT, 240);
      this->camRight->set(CAP_PROP_FPS, 30);
      this->camRight->set(CAP_PROP_AUTO_EXPOSURE, 0.25);
      this->camRight->set(CAP_PROP_AUTOFOCUS, 0);
      this->camRight->set(CAP_PROP_EXPOSURE, -4);
      this->camRight->set(CAP_PROP_MODE, CAP_OPENCV_MJPEG);
   }
}

vector<TopicInfo> NetworkManager::getROSTopicList()
{
   ros::master::V_TopicInfo topic_infos;
   ros::master::getTopics(topic_infos);
   vector<TopicInfo> names;
   for (int i = 0; i < topic_infos.size(); i++)
   {
      names.emplace_back(topic_infos[i]);
   }
   return names;
}

void NetworkManager::depthCallback(const ImageConstPtr& depthMsg)
{
   ROS_DEBUG("Depth Callback", depthMsg->header.seq);
   depthMessage = depthMsg;
}

void NetworkManager::colorCallback(const sensor_msgs::ImageConstPtr& colorMsg, String name)
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
   paramsAvailable = true;
   ROS_DEBUG("PARAMS CALLBACK");
}

void NetworkManager::load_next_stereo_frame(Mat& left, Mat& right, ApplicationState& app)
{
   camLeft->read(left);
   camRight->read(right);
}

void NetworkManager::load_next_frame(Mat& depth, Mat& color, double& timestamp, ApplicationState& app)
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
         timestamp = depthMessage.get()->header.stamp.toSec();

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
         img_ptr_color = cv_bridge::toCvCopy(*colorMessage, image_encodings::MONO8);
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

void NetworkManager::init_ros_node(int argc, char **argv, ApplicationState& app)
{
   ROS_DEBUG("Starting ROS Node");
   init(argc, argv, "PlanarRegionPublisher");
   rosNode = new NodeHandle();

   // ROSTopic Publishers
   planarRegionPub = rosNode->advertise<map_sense::RawGPUPlanarRegionList>("/map/regions/test", 3);
   slamPosePub = rosNode->advertise<geometry_msgs::PoseStamped>("/mapsense/slam/pose", 3);

   // ROSTopic Subscribers
   string depthTopicName = app.DEPTH_ALIGNED ? "/" + app.TOPIC_CAMERA_NAME + "/aligned_depth_to_color/image_raw" : "/" + app.TOPIC_CAMERA_NAME +
                                                                                                                             "/depth/image_rect_raw";
   string depthInfoTopicName = app.DEPTH_ALIGNED ? "/" + app.TOPIC_CAMERA_NAME + "/aligned_depth_to_color/camera_info" : "/" +
                                                                                                                                   app.TOPIC_CAMERA_NAME +
                                                                                                                                   "/depth/camera_info";

   addReceiver(TopicInfo(depthTopicName, "sensor_msgs/Image"), TopicInfo(depthInfoTopicName, "sensor_msgs/CameraInfo"));

   //   subDepth = rosNode->subscribe(depthTopicName, 3, &NetworkManager::depthCallback, this);
   //   subDepthCamInfo = rosNode->subscribe(depthInfoTopicName, 2, &NetworkManager::depthCameraInfoCallback, this);

   //   subColor = rosNode->subscribe("/camera/image_mono", 3, &NetworkManager::colorCallback, this); /* "/" + app.TOPIC_CAMERA_NAME + "/color/image_raw" */
   //   subColorCompressed = rosNode->subscribe("/" + app.TOPIC_CAMERA_NAME + "/color/image_raw/compressed", 3, &NetworkManager::colorCompressedCallback, this);
   //   subColorCamInfo = rosNode->subscribe("/" + app.TOPIC_CAMERA_NAME + "/color/camera_info", 2, &NetworkManager::colorCameraInfoCallback, this);
   //
   subMapSenseParams = rosNode->subscribe("/map/config", 8, &NetworkManager::mapSenseParamsCallback, this);

   ROS_DEBUG("Started ROS Node");
}

void NetworkManager::acceptMapsenseConfiguration(ApplicationState& appState)
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

int NetworkManager::addReceiver(TopicInfo data, TopicInfo info)
{
   ROS_DEBUG("Adding Receiver: (%s), (%s)", data.name.c_str(), info.name.c_str());
   ROS1TopicReceiver *receiver = nullptr;
   if (data.datatype == "sensor_msgs/Image")
      receiver = new ImageReceiver(rosNode, data.name, info.name, false);
   if (data.datatype == "sensor_msgs/CompressedImage")
      receiver = new ImageReceiver(rosNode, data.name, info.name, true);

   if (receiver != nullptr)
   {
      receiver->setAppUtils(this->appUtils);
      receivers.emplace_back(receiver);
   } else
   {
      printf("Request to add receiver: %s\n", data.name.c_str());
   }
   ROS_DEBUG("Receiver Added Successfully");
   return receivers.size() - 1;
}

void NetworkManager::getTopicSelection(vector<TopicInfo> topics, TopicInfo& currentTopic)
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

void NetworkManager::ImGuiUpdate()
{
   /* Testing Combo Drop Downs */
   vector<TopicInfo> topics = getROSTopicList();
   getTopicSelection(topics, currentDataTopic);
   if (ImGui::Button("Add Receiver"))
      addReceiver(currentDataTopic);
   for (int i = 0; i < receivers.size(); i++)
      receivers[i]->ImGuiUpdate();
}

void NetworkManager::receiverUpdate(ApplicationState& app)
{
   for (int i = 0; i < receivers.size(); i++)
   {
      ROS_DEBUG("RenderUpdate");
      receivers[i]->processMessage(app);
      receivers[i]->render();
   }
}

void NetworkManager::spin_ros_node()
{
   ROS_DEBUG("SpinOnce");
   spinOnce();
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

void NetworkManager::publishSLAMPose(RigidBodyTransform worldToSensorTransform)
{
   Quaterniond quaternion = worldToSensorTransform.getQuaternion();
   Vector3d position = worldToSensorTransform.getTranslation();

   geometry_msgs::PoseStamped pose;
   pose.pose.position = geometry_msgs::Point();
   pose.pose.position.x = position.x();
   pose.pose.position.y = position.y();
   pose.pose.position.z = position.z();

   pose.pose.orientation = geometry_msgs::Quaternion();
   pose.pose.orientation.x = quaternion.x();
   pose.pose.orientation.y = quaternion.y();
   pose.pose.orientation.z = quaternion.z();
   pose.pose.orientation.w = quaternion.w();

   this->slamPosePub.publish(pose);
}
