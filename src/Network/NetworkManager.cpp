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

         /*
            image_width: 1280
            image_height: 1024
            camera_name: 12502226
            camera_matrix: (3, 3): data: [305.6744323977819, 0, 640.9175104495503, 0, 307.6945377439216, 512.9095529465322, 0, 0, 1]
            distortion_model: rational_polynomial
            distortion_coefficients: (1, 8): data: [-0.1304880574839372, 0.0343337720836711, 0, 0, 0.002347490605947351, 0.163868408051474, -0.02493286434834704, 0.01394671162254435]
            rectification_matrix: (3, 3): data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
            projection_matrix: (3, 4): data: [59.01958465576172, 0, 640.8088034578032, 0, 0, 72.80045318603516, 513.8869098299074, 0, 0, 0, 1, 0]
         */

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
   nh = new NodeHandle();

   // ROSTopic Publishers
   planarRegionPub = nh->advertise<map_sense::RawGPUPlanarRegionList>("/map/regions/test", 3);

   // ROSTopic Subscribers
   string depthTopicName = app.DEPTH_ALIGNED ? "/" + app.TOPIC_CAMERA_NAME + "/aligned_depth_to_color/image_raw" : "/" + app.TOPIC_CAMERA_NAME +
                                                                                                                   "/depth/image_rect_raw";
   string depthInfoTopicName = app.DEPTH_ALIGNED ? "/" + app.TOPIC_CAMERA_NAME + "/aligned_depth_to_color/camera_info" : "/" + app.TOPIC_CAMERA_NAME +
                                                                                                                         "/depth/camera_info";
   //   subDepth = nh->subscribe(depthTopicName, 3, &NetworkManager::depthCallback, this);
   //   subDepthCamInfo = nh->subscribe(depthInfoTopicName, 2, &NetworkManager::depthCameraInfoCallback, this);

   //   subColor = nh->subscribe("/camera/image_mono", 3, &NetworkManager::colorCallback, this); /* "/" + app.TOPIC_CAMERA_NAME + "/color/image_raw" */
   //   subColorCompressed = nh->subscribe("/" + app.TOPIC_CAMERA_NAME + "/color/image_raw/compressed", 3, &NetworkManager::colorCompressedCallback, this);
   //   subColorCamInfo = nh->subscribe("/" + app.TOPIC_CAMERA_NAME + "/color/camera_info", 2, &NetworkManager::colorCameraInfoCallback, this);
   //
   subMapSenseParams = nh->subscribe("/map/config", 8, &NetworkManager::mapSenseParamsCallback, this);

   ROS_DEBUG("Started ROS Node");
}

void NetworkManager::addReceiver(TopicInfo data, TopicInfo info)
{

   ROS1TopicReceiver *receiver = nullptr;
   if (data.datatype == "sensor_msgs/Image")
      receiver = new ImageReceiver(nh, data.name, info.name, false);
   if (data.datatype == "sensor_msgs/CompressedImage")
      receiver = new ImageReceiver(nh, data.name, info.name, true);

   if (receiver != nullptr)
   {
      receiver->setAppUtils(this->appUtils);
      receivers.emplace_back(receiver);
   }else
   {
      printf("Request to add receiver: %s\n", data.name.c_str());
   }
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
      ROS_INFO("RenderUpdate");
      receivers[i]->processMessage(app);
      receivers[i]->render();
   }
}

void NetworkManager::spin_ros_node()
{
   ROS_DEBUG("SpinOnce");
   spinOnce();
}
