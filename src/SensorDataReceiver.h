#ifndef SRC_SENSORDATARECEIVER_H
#define SRC_SENSORDATARECEIVER_H

#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include "map_sense/PlanarRegions.h"
#include "geometry_msgs/PoseStamped.h"

#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "cv_bridge/cv_bridge.h"

#include <iostream>
#include <CL/cl.hpp>
#include "math.h"
#include <sstream>
#include <random>

using namespace ros;
using namespace std;
using namespace chrono;
using namespace cv;
using namespace sensor_msgs;

class SensorDataReceiver {
public:
    ImageConstPtr colorMessage;
    ImageConstPtr depthMessage;
    NodeHandle* nh;
    Subscriber subDepth;
    Subscriber subColor;


    bool nextDepthAvailable = false;
    bool nextColorAvailable = false;

    void get_sample_depth(Mat depth, float mean, float stddev);
    void get_sample_color(Mat color);

    void load_sample_depth(String filename, Mat& depth);
    void load_next_frame(Mat& depth, Mat& color);
    void load_sample_color(String filename, Mat& color);

    void depthCallback(const ImageConstPtr &depthMsg);
    void colorCallback(const sensor_msgs::ImageConstPtr &colorMsg);

    void init_ros_node(int argc, char **argv);
    void spin_ros_node();


};


#endif //SRC_SENSORDATARECEIVER_H
