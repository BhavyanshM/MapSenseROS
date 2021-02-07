#include "NetworkManager.h"

void NetworkManager::depthCallback(const ImageConstPtr &depthMsg) {
    ROS_INFO("Depth Callback", depthMsg->header.seq);
    depthMessage = depthMsg;
}

void NetworkManager::colorCallback(const sensor_msgs::ImageConstPtr &colorMsg) {
    colorMessage = colorMsg;
    ROS_INFO("COLOR CALLBACK");
}

void NetworkManager::colorCompressedCallback(const sensor_msgs::CompressedImageConstPtr &compressedMsg) {
    colorCompressedMessage = compressedMsg;
    ROS_INFO("COLOR CALLBACK");
}

void NetworkManager::mapSenseParamsCallback(const map_sense::MapSenseParams msg) {
    paramsMessage = msg;
    ROS_INFO("COLOR CALLBACK");
}

void NetworkManager::load_next_frame(Mat &depth, Mat &color) {
    cv_bridge::CvImagePtr img_ptr_depth;
    cv_bridge::CvImagePtr img_ptr_color;
    cv_bridge::CvImagePtr img_ptr_color_compressed;
    ROS_INFO("Process Data Callback");
    if (depthMessage != nullptr) {
        try {
            ROS_INFO("Callback: Depth:%d", depthMessage->header.stamp.sec);
            img_ptr_depth = cv_bridge::toCvCopy(*depthMessage, image_encodings::TYPE_16UC1);
            depth = img_ptr_depth->image;
            pyrDown(depth, depth);
            ROS_INFO("INPUT_DEPTH:(%d,%d)", depth.rows, depth.cols);

        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("Could not convert to image!");
        }
    }
    if (colorMessage != nullptr) {
        try {
            ROS_INFO("Callback: Color:%d", colorMessage->header.stamp.sec);
            img_ptr_color = cv_bridge::toCvCopy(*colorMessage, image_encodings::TYPE_8UC3);
            color = img_ptr_color->image;
            pyrDown(color, color);

        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("Could not convert to image!");
        }
    } else if (colorCompressedMessage != nullptr) {
        try {
            ROS_INFO("Callback: CompressedColor:%d", colorCompressedMessage->header.stamp.sec);
//            img_ptr_color = cv_bridge::toCvCopy(*colorCompressedMessage, image_encodings::TYPE_8UC3);
            color = imdecode(cv::Mat(colorCompressedMessage->data), 1);

        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("Could not convert compressedImage to image!");
        }
    }
    ROS_INFO("Data Frame Loaded");
}

void NetworkManager::init_ros_node(int argc, char **argv) {
    ROS_INFO("Starting ROS Node");
    init(argc, argv, "PlanarRegionPublisher");
    nh = new NodeHandle();

    planarRegionPub = nh->advertise<map_sense::RawGPUPlanarRegionList>("/map/regions/test", 10);
    subDepth = nh->subscribe("/camera/depth/image_rect_raw", 8, &NetworkManager::depthCallback, this);
    subColor = nh->subscribe("/camera/color/image_raw", 8, &NetworkManager::colorCallback, this);
    subColorCompressed = nh->subscribe("/camera/color/image_raw/compressed", 8,
                                       &NetworkManager::colorCompressedCallback, this);
//    subMapSenseParams = nh->subscribe("/map_sense/params", 8, &NetworkManager::colorCompressedCallback, this);
    ROS_INFO("Started ROS Node");
}

void NetworkManager::spin_ros_node() {
    ROS_INFO("SpinOnce");
    spinOnce();
}


void NetworkManager::load_sample_depth(String filename, Mat &depth) {
    depth = imread(ros::package::getPath("map_sense") + filename, IMREAD_ANYDEPTH);
}

void NetworkManager::load_sample_color(String filename, Mat &color) {
    color = imread(ros::package::getPath("map_sense") + filename, IMREAD_COLOR);
}

void NetworkManager::get_sample_depth(Mat depth, float mean, float stddev) {
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(mean, stddev);
    for (int i = 0; i < depth.cols; i++) {
        for (int j = 0; j < depth.rows; j++) {
            float d = 10.04;
            d += distribution(generator);
            if (160 < i && i < 350 && 200 < j && j < 390) {
                // d = 0.008 * i + 0.014 * j;
                depth.at<short>(j, i) = (d - 2.0f) * 1000;
            } else {
                depth.at<short>(j, i) = d * 1000;
            }
        }
    }
}

void NetworkManager::get_sample_color(Mat color) {
    for (int i = 0; i < color.rows; i++) {
        for (int j = 0; j < color.cols; j++) {
            color.at<Vec3b>(i, j) = Vec3b(0, 0, 255);
        }
    }
}