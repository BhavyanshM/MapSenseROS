#include "SensorDataReceiver.h"

void SensorDataReceiver::depthCallback(const ImageConstPtr &depthMsg) {
    ROS_INFO("Depth Callback", depthMsg->header.seq);
    depthMessage = depthMsg;
}

void SensorDataReceiver::colorCallback(const sensor_msgs::ImageConstPtr &colorMsg) {
    colorMessage = colorMsg;
}

void SensorDataReceiver::load_next_frame(Mat& depth, Mat& color){
    cv_bridge::CvImagePtr img_ptr_depth;
    cv_bridge::CvImagePtr img_ptr_color;
    ROS_INFO("Process Data Callback");
    if (depthMessage != nullptr) {
        try {
            ROS_INFO("Callback: Depth:%d", depthMessage->header.stamp.sec);
            img_ptr_depth = cv_bridge::toCvCopy(*depthMessage, image_encodings::TYPE_16UC1);
            depth = img_ptr_depth->image;

        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("Could not convert to image!");
        }
    }
    if (colorMessage != nullptr ) {
        try {
            ROS_INFO("Callback: Color:%d", colorMessage->header.stamp.sec);
            img_ptr_color = cv_bridge::toCvCopy(*colorMessage, image_encodings::TYPE_8UC3);
            color = img_ptr_color->image;

        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("Could not convert to image!");
        }
    }
    ROS_INFO("Data Frame Loaded");
}

void SensorDataReceiver::init_ros_node(int argc, char **argv) {
    ROS_INFO("Starting ROS Node");
    init(argc, argv, "PlanarRegionPublisher");
    nh = new NodeHandle();

    planarRegionPub = nh->advertise<map_sense::RawGPUPlanarRegionList>("/map/regions/test", 10);
    subDepth = nh->subscribe("/camera/depth/image_rect_raw", 8, &SensorDataReceiver::depthCallback, this);
    subColor = nh->subscribe("/camera/color/image_raw", 8, &SensorDataReceiver::colorCallback, this);
    ROS_INFO("Started ROS Node");
}

void SensorDataReceiver::spin_ros_node() {
    ROS_INFO("SpinOnce");
    spinOnce();
}


void SensorDataReceiver::load_sample_depth(String filename, Mat& depth){
    depth = imread(ros::package::getPath("map_sense") + filename, IMREAD_ANYDEPTH);
}

void SensorDataReceiver::load_sample_color(String filename, Mat& color){
    color = imread(filename, IMREAD_ANYCOLOR);
}

void SensorDataReceiver::get_sample_depth(Mat depth, float mean, float stddev) {
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

void SensorDataReceiver::get_sample_color(Mat color) {
    for (int i = 0; i < color.rows; i++) {
        for (int j = 0; j < color.cols; j++) {
            color.at<Vec3b>(i, j) = Vec3b(0, 0, 255);
        }
    }
}